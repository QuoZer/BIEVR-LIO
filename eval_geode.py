#!/usr/bin/env python3
"""
Evaluate BIEVR-LIO trajectories against GEODE ground truth.

Mirrors GLIM's eval_geode.py. Pipeline (follows GEODE README for the gamma /
Livox AVIA device, and extended for the alpha / Velodyne VLP-16 device on
`urban_tunnel2`):
  1. (optional) Fix root-owned bievr_out files (sudo chown).
  2. Bring BIEVR's trajectory into the LiDAR frame, then apply the device→GT
     frame transform (replicates gamma2GT_leica.py / alpha2GT_gnss.py).
  3. Compute APE RMSE via evo_ape tum.

Note vs GLIM: BIEVR-LIO logs the IMU pose (T_W_I), whereas GLIM's traj_lidar.txt
is the LiDAR pose. GEODE's device→GT extrinsic is referenced to the LiDAR frame,
so by default we first compose T_W_L = T_W_I @ T_I_L using the LiDAR→IMU
extrinsic from the sensor config, then follow the exact same steps as GLIM.

For gamma (metro_tunnel), the IMU is co-located with the LiDAR (T_I_L =
identity) so this composition is a no-op and evaluating in the "lidar" vs
"imu" body frame gives identical numbers. For alpha (urban_tunnel2) there is a
real 0.303 m lever arm between LiDAR and IMU (see config/sensor_configs/
geode_alpha.yaml), so which body frame the device→GT extrinsic is anchored to
actually matters (GEODE brief ASSUMPTION A3) — pass --body-frame to control
this, or leave the default "both" to see both numbers side by side.

Usage:
  python3 eval_geode.py [tunnel1] [tunnel2] [shield1] [shield5] [urban_tunnel2]
  (no args = all five sequences)
"""

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
from pyquaternion import Quaternion

SCRIPT_DIR = Path(__file__).resolve().parent

# Two independent GEODE dataset roots (metro_tunnel = gamma/Leica-GT sequences,
# urban_tunnel = alpha/GNSS-GT sequences). Both are env-overridable so a second
# hardcoded path never needs to live here.
DATASET_ROOT_METRO = Path(os.environ.get(
    "DATASET_ROOT", "/home/roser/Documents/Datasets/geode/metro_tunnel"))
DATASET_ROOT_URBAN = Path(os.environ.get(
    "URBAN_DATASET_ROOT", "/home/roser/Documents/Datasets/geode/urban_tunnel"))

# Gamma (carol) → GT extrinsic, taken verbatim from gamma2GT_leica.py.
_GAMMA_QW, _GAMMA_QX, _GAMMA_QY, _GAMMA_QZ = 0.999901, -0.00492765, 0.00575961, 0.0117651
_GAMMA_T = np.array([0.00947221, -0.308202, -0.365733])

# Alpha → GNSS-GT extrinsic, taken from the uncommented block of
# GEODE_dataset/script/alpha2GT_gnss.py (brief §5).
#
# ⚠ ASSUMPTION A1: that block is labelled `# bob` (beta's alias) in the
# upstream source, but is used here as the genuine alpha→GNSS-GT extrinsic
# because (a) the file is named alpha2GT_gnss.py and (b) urban_tunnel2's GT is
# GNSS-derived like this function's name implies; we treat "# bob" as a
# copy-paste leftover comment, not a semantic label. The extrinsic legitimately
# differs from gamma's because it's anchored to a different GT modality
# (GNSS/INS vs Leica total station), not because it's a different device.
_ALPHA_QW, _ALPHA_QX, _ALPHA_QY, _ALPHA_QZ = 0.9999909, -0.0009, -0.00355, 0.0022
_ALPHA_T = np.array([0.0090, -0.2925, 0.4533])

GT_FILES = {
    "tunnel1": "Tunneling_tunnel1.txt",
    "tunnel2": "Tunneling_tunnel2.txt",
    "shield1": "Shield_tunnel1.txt",
    "shield5": "Shield_tunnel5.txt",
    "urban_tunnel2": "Urban_Tunnel02.txt",
}

SEQ_DATASET_ROOT = {
    "tunnel1": DATASET_ROOT_METRO,
    "tunnel2": DATASET_ROOT_METRO,
    "shield1": DATASET_ROOT_METRO,
    "shield5": DATASET_ROOT_METRO,
    "urban_tunnel2": DATASET_ROOT_URBAN,
}

# Default sensor config per sequence, only used to load T_I_L (LiDAR→IMU) for
# --body-frame lidar. Overridable per-invocation via --sensor-config (applies
# uniformly to every sequence passed that run).
SEQ_SENSOR_CONFIG = {
    "tunnel1": "geode.yaml",
    "tunnel2": "geode.yaml",
    "shield1": "geode.yaml",
    "shield5": "geode.yaml",
    "urban_tunnel2": "geode_alpha.yaml",
}


def build_extrinsic(seq: str) -> np.ndarray:
    """Device→GT extrinsic for `seq` (gamma→Leica-GT or alpha→GNSS-GT)."""
    if seq == "urban_tunnel2":
        qw, qx, qy, qz = _ALPHA_QW, _ALPHA_QX, _ALPHA_QY, _ALPHA_QZ
        t_vec = _ALPHA_T
    else:
        qw, qx, qy, qz = _GAMMA_QW, _GAMMA_QX, _GAMMA_QY, _GAMMA_QZ
        t_vec = _GAMMA_T
    R = Quaternion(qw, qx, qy, qz).rotation_matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t_vec
    return T


def load_T_imu_lidar(sensor_config: Path) -> np.ndarray:
    """Load T_I_L (LiDAR→IMU) from a BIEVR sensor config's `calibration` block."""
    with open(sensor_config) as f:
        cfg = yaml.safe_load(f)
    calib = cfg["calibration"]
    T = np.eye(4)
    T[:3, :3] = np.array(calib["rotation"], dtype=float).reshape(3, 3)
    T[:3, 3] = np.array(calib["translation"], dtype=float)
    return T


def transform_trajectory(src: Path, dst: Path, T_extrinsic: np.ndarray,
                         T_imu_lidar: np.ndarray, body_frame: str) -> int:
    """Write TUM poses in the GT frame.

    body_frame == "lidar": T_eval = (T_W_I @ T_I_L) @ inv(T_extrinsic)  (existing
        convention: compose to the LiDAR frame first, then apply an extrinsic
        that is anchored to the LiDAR frame.)
    body_frame == "imu":   T_eval = T_W_I @ inv(T_extrinsic)  (apply the
        extrinsic directly to BIEVR's native IMU-frame trajectory — only a
        meaningful alternative when T_extrinsic might instead be anchored to
        the IMU frame; see ASSUMPTION A3 in the module docstring / brief §5.)
    """
    assert body_frame in ("lidar", "imu")
    T_inv = np.linalg.inv(T_extrinsic)
    n = 0
    with open(src) as f_in, open(dst, "w") as f_out:
        for line in f_in:
            d = line.split()
            if not d:
                continue
            ts = d[0]
            tx, ty, tz = float(d[1]), float(d[2]), float(d[3])
            qx, qy, qz, qw = float(d[4]), float(d[5]), float(d[6]), float(d[7])

            R_imu = Quaternion(qw, qx, qy, qz).rotation_matrix
            T_W_I = np.eye(4)
            T_W_I[:3, :3] = R_imu
            T_W_I[:3, 3] = [tx, ty, tz]

            if body_frame == "lidar":
                T_dev = T_W_I @ T_imu_lidar  # IMU pose → LiDAR pose (GLIM's frame)
            else:
                T_dev = T_W_I  # evaluate BIEVR's native IMU-frame trajectory directly

            T_eval = T_dev @ T_inv

            t_out = T_eval[:3, 3]
            q_out = Quaternion(matrix=T_eval[:3, :3])
            # TUM format: ts x y z qx qy qz qw
            f_out.write(
                f"{ts} {t_out[0]:.6f} {t_out[1]:.6f} {t_out[2]:.6f} "
                f"{q_out[1]:.6f} {q_out[2]:.6f} {q_out[3]:.6f} {q_out[0]:.6f}\n"
            )
            n += 1
    return n


def trajectory_path_length(tum_path: Path) -> float:
    """Cumulative Euclidean path length of a TUM trajectory file [m]."""
    pts = []
    with open(tum_path) as f:
        for line in f:
            d = line.split()
            if not d:
                continue
            pts.append((float(d[1]), float(d[2]), float(d[3])))
    if len(pts) < 2:
        return 0.0
    arr = np.array(pts)
    return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))


def evo_ape_rmse(est: Path, gt: Path, t_offset: float):
    """Run evo_ape tum at a given t_offset; return (APE RMSE [m], n_associated_pairs)."""
    result = subprocess.run(
        [
            "evo_ape", "tum",
            str(gt), str(est),
            "-a",
            "--t_max_diff", "0.1",
            "--t_offset", f"{t_offset}",
            "-v",
        ],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        return None, None
    m = re.search(r"rmse\s+([\d.]+)", result.stdout)
    n_m = re.search(r"Compared (\d+) absolute pose pairs", result.stdout)
    rmse = float(m.group(1)) if m else None
    n_pairs = int(n_m.group(1)) if n_m else None
    return rmse, n_pairs


def _frange(lo: float, hi: float, step: float):
    n = int(round((hi - lo) / step))
    return [round(lo + i * step, 6) for i in range(n + 1)]


def sweep_offset(est: Path, gt: Path, lo: float, hi: float,
                 coarse: float, fine: float, extra_fine: float):
    """Find the t_offset minimizing APE RMSE (GEODE rmse.py methodology).

    GEODE's Leica GT is not tightly time-synced to the sensor clock; the offset
    is constant per sequence but differs between sequences, so it must be
    calibrated by minimizing the APE. The GNSS GT (urban_tunnel2) is expected to
    be closer to GPS-time synced, but the sequence is driven at ~14 m/s so
    residual sub-second offset still matters — hence the extra-fine 0.01 s
    refinement stage on top of the original coarse+fine sweep (brief §6).

    Returns (rmse, offset, n_pairs, hit_edge) where hit_edge is True if the
    coarse-stage winner sits at the ±(hi/lo) sweep boundary, which signals a
    non-converged sweep (real signal outside ±[lo, hi]) rather than a trustworthy
    result.
    """
    def best_over(offsets):
        vals = [(o,) + evo_ape_rmse(est, gt, o) for o in offsets]
        vals = [(o, r, n) for o, r, n in vals if r is not None]
        return min(vals, key=lambda x: x[1]) if vals else (None, None, None)

    coarse_offsets = _frange(lo, hi, coarse)
    o_c, r_c, n_c = best_over(coarse_offsets)
    if o_c is None:
        return None, None, None, False
    hit_edge = abs(o_c - lo) < 1e-9 or abs(o_c - hi) < 1e-9

    o_f, r_f, n_f = best_over(_frange(o_c - coarse, o_c + coarse, fine))
    if o_f is None:
        o_f, r_f, n_f = o_c, r_c, n_c

    o_e, r_e, n_e = best_over(_frange(o_f - fine, o_f + fine, extra_fine))
    if o_e is None:
        o_e, r_e, n_e = o_f, r_f, n_f

    return r_e, o_e, n_e, hit_edge


def fix_ownership(path: Path) -> None:
    uid = os.getuid()
    gid = os.getgid()
    print(f"  sudo chown -R {uid}:{gid} {path}")
    subprocess.run(["sudo", "chown", "-R", f"{uid}:{gid}", str(path)], check=True)


def evaluate_sequence(seq: str, dataset_root: Path, T_ext: np.ndarray,
                      T_imu_lidar: np.ndarray, fix_owner: bool, args,
                      run_dir: Path | None = None) -> dict:
    seq_dir = dataset_root / seq
    bievr_out = run_dir if run_dir is not None else seq_dir / "bievr_out"
    src_traj = bievr_out / "traj_lidar.txt"
    gt_file = dataset_root / GT_FILES[seq]

    results: dict = {}

    if not src_traj.exists():
        print(f"[{seq}] traj_lidar.txt not found in {bievr_out} — skipping.")
        return results
    if not gt_file.exists():
        print(f"[{seq}] GT file {gt_file} not found — skipping.")
        return results

    if fix_owner:
        fix_ownership(bievr_out)

    body_frames = ["lidar", "imu"] if args.body_frame == "both" else [args.body_frame]

    for body_frame in body_frames:
        transformed = bievr_out / f"traj_{body_frame}_in_gt_frame.txt"
        n = transform_trajectory(src_traj, transformed, T_ext, T_imu_lidar, body_frame)
        path_len = trajectory_path_length(transformed)
        print(f"[{seq}/{body_frame}] Transformed {n} poses → {transformed.name} "
              f"(path length {path_len:.1f} m)")

        if args.t_offset is not None:
            rmse, n_pairs = evo_ape_rmse(transformed, gt_file, args.t_offset)
            offset = args.t_offset
            hit_edge = False
        else:
            rmse, offset, n_pairs, hit_edge = sweep_offset(
                transformed, gt_file, args.t_offset_min, args.t_offset_max,
                args.t_offset_coarse, args.t_offset_fine, args.t_offset_extra_fine)

        if rmse is not None:
            edge_note = "  ⚠ SWEEP HIT EDGE — treat as non-convergence, not a result" if hit_edge else ""
            print(f"[{seq}/{body_frame}] APE RMSE: {rmse:.4f} m  "
                  f"(t_offset = {offset:g} s, {n_pairs} associated pairs, "
                  f"path length {path_len:.1f} m){edge_note}")
        else:
            print(f"[{seq}/{body_frame}] evo_ape failed — check that GT/traj overlap in time.")

        results[body_frame] = {
            "rmse": rmse, "offset": offset, "n_pairs": n_pairs,
            "path_length": path_len, "hit_edge": hit_edge,
        }

    if len(results) == 2 and all(r["rmse"] is not None for r in results.values()):
        lidar_r, imu_r = results["lidar"]["rmse"], results["imu"]["rmse"]
        winner = "lidar" if lidar_r <= imu_r else "imu"
        print(f"[{seq}] body-frame comparison: lidar={lidar_r:.4f} m vs imu={imu_r:.4f} m "
              f"→ '{winner}' wins by {abs(lidar_r - imu_r):.4f} m")

    return results


def main():
    parser = argparse.ArgumentParser(description="Evaluate BIEVR-LIO on GEODE sequences")
    parser.add_argument("sequences", nargs="*", default=list(GT_FILES.keys()),
                        help=f"Sequences to evaluate (default: all). Choices: {list(GT_FILES.keys())}")
    parser.add_argument("--sensor-config", type=Path, default=None,
                        help="Sensor config providing T_I_L (LiDAR→IMU extrinsic). "
                             "Default: per-sequence (geode.yaml for metro_tunnel, "
                             "geode_alpha.yaml for urban_tunnel2). Overriding applies "
                             "to every sequence passed this run.")
    parser.add_argument("--body-frame", choices=["lidar", "imu", "both"], default="both",
                        help="Which body frame to anchor the device→GT extrinsic to "
                             "(ASSUMPTION A3). 'lidar' composes T_W_L = T_W_I @ T_I_L "
                             "first (existing convention); 'imu' evaluates BIEVR's "
                             "native T_W_I directly. Default 'both' reports both and "
                             "prints the winner. For gamma sequences T_I_L is identity "
                             "so both give the same number.")
    parser.add_argument("--fix-ownership", action="store_true",
                        help="sudo chown bievr_out dirs back to current user (usually unneeded).")
    parser.add_argument("--run-dir", type=Path, default=None,
                        help="Override the default <seq>/bievr_out directory: read "
                             "traj_lidar.txt from here and write the *_in_gt_frame.txt "
                             "files here too. Only makes sense with a single sequence "
                             "argument.")
    # GEODE's Leica GT is not tightly time-synced to the sensor clock; the offset
    # is constant per sequence but differs between sequences. Following GEODE's
    # rmse.py, we sweep t_offset and report the minimum APE. Pass --t-offset to
    # pin a fixed value instead.
    parser.add_argument("--t-offset", type=float, default=None,
                        help="Fixed GT time offset [s]; skips the sweep.")
    parser.add_argument("--t-offset-min", type=float, default=-5.0)
    parser.add_argument("--t-offset-max", type=float, default=5.0)
    parser.add_argument("--t-offset-coarse", type=float, default=0.5)
    parser.add_argument("--t-offset-fine", type=float, default=0.05)
    parser.add_argument("--t-offset-extra-fine", type=float, default=0.01,
                        help="Third refinement stage after coarse+fine (brief §6): "
                             "matters on fast (~14 m/s) sequences where 0.05 s of "
                             "residual offset is ~0.7 m of spurious APE.")
    args = parser.parse_args()

    if args.run_dir is not None and len(args.sequences) > 1:
        print("error: --run-dir only makes sense with a single sequence argument",
              file=sys.stderr)
        sys.exit(1)

    results = {}
    for seq in args.sequences:
        print(f"\n── {seq} ──────────────────────────────")
        dataset_root = SEQ_DATASET_ROOT[seq]
        sensor_config = args.sensor_config or (
            SCRIPT_DIR / "config" / "sensor_configs" / SEQ_SENSOR_CONFIG[seq])
        T_ext = build_extrinsic(seq)
        T_imu_lidar = load_T_imu_lidar(sensor_config)
        results[seq] = evaluate_sequence(seq, dataset_root, T_ext, T_imu_lidar,
                                         args.fix_ownership, args, run_dir=args.run_dir)

    print("\n── Summary ──────────────────────────────")
    for seq, per_frame in results.items():
        if not per_frame:
            print(f"  {seq:<14} failed / skipped")
            continue
        for body_frame, r in per_frame.items():
            val = f"{r['rmse']:.4f} m" if r["rmse"] is not None else "failed"
            edge = " (EDGE)" if r["hit_edge"] else ""
            print(f"  {seq:<14} [{body_frame:<5}] APE RMSE = {val}  "
                  f"@ t_offset={r['offset']}{edge}  n_pairs={r['n_pairs']}  "
                  f"path={r['path_length']:.1f} m")


if __name__ == "__main__":
    main()
