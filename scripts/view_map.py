#!/usr/bin/env python3
"""Visualize a BIEVR-LIO map export.

Works on either the exported .pcd or the native .bumpmap (via load_bumpmap.py).

Usage (needs Open3D — e.g. the kiss-slam venv):
    ~/repos/kiss-slam-venv/bin/python scripts/view_map.py <map.pcd|map.bumpmap> [--color height|intensity]
    # inspect the raw per-voxel bump images (2-D height patches) instead of the cloud:
    ~/repos/kiss-slam-venv/bin/python scripts/view_map.py <map.bumpmap> --patches 25

Controls in the Open3D window: drag to rotate, scroll to zoom, [ ] to size points.
"""
import argparse
import struct
import sys
from pathlib import Path

import numpy as np


def read_pcd_xyzi(path):
    """Minimal reader for the binary PCD written by exportMap (x y z intensity, float32)."""
    with open(path, "rb") as f:
        header, n = [], 0
        while True:
            line = f.readline().decode("ascii", "replace")
            header.append(line)
            if line.startswith("POINTS"):
                n = int(line.split()[1])
            if line.startswith("DATA"):
                if "binary" not in line:
                    raise SystemExit("Only DATA binary PCDs are supported by this quick reader.")
                break
        data = np.frombuffer(f.read(n * 16), dtype=np.float32).reshape(n, 4)
    return data[:, :3].astype(np.float64), data[:, 3].astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help=".pcd or .bumpmap")
    ap.add_argument("--color", choices=["height", "intensity"], default="height")
    ap.add_argument("--patches", type=int, default=0,
                    help="bumpmap only: show N per-voxel bump images as 2-D heatmaps instead of the 3-D cloud")
    args = ap.parse_args()
    p = Path(args.path)

    # --- per-voxel bump-image montage (the distinctive representation) ---
    if args.patches > 0:
        if p.suffix != ".bumpmap":
            raise SystemExit("--patches needs the native .bumpmap file.")
        import matplotlib.pyplot as plt
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from load_bumpmap import load_bumpmap
        hdr, voxels = load_bumpmap(p)
        # pick the voxels with the most observed pixels (most informative patches)
        voxels = sorted(voxels, key=lambda v: int((v["bump_weights"] > 0).sum()), reverse=True)
        n = min(args.patches, len(voxels))
        cols = int(np.ceil(np.sqrt(n)))
        rows = int(np.ceil(n / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(2 * cols, 2 * rows))
        for ax in np.ravel(axes):
            ax.axis("off")
        for k in range(n):
            v = voxels[k]
            img = np.where(v["bump_weights"] > 0, v["bump_img"], np.nan)
            ax = np.ravel(axes)[k]
            ax.imshow(img, cmap="viridis", origin="lower")
            ax.set_title(f"{int((v['bump_weights']>0).sum())} px", fontsize=7)
        fig.suptitle(f"{p.name}: {n} voxel bump images (height, m)")
        plt.tight_layout()
        plt.show()
        return

    # --- 3-D point cloud view ---
    import open3d as o3d
    if p.suffix == ".pcd":
        xyz, inten = read_pcd_xyzi(p)
    elif p.suffix == ".bumpmap":
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from load_bumpmap import load_bumpmap, reconstruct_points
        hdr, voxels = load_bumpmap(p)
        pts = reconstruct_points(voxels, hdr["px_size"])  # (N,4): x y z weight
        xyz, inten = pts[:, :3], pts[:, 3]
    else:
        raise SystemExit("Give a .pcd or .bumpmap file.")

    print(f"{p.name}: {len(xyz):,} points  bbox {xyz.min(0).round(1)} .. {xyz.max(0).round(1)}")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    if args.color == "height":
        v = xyz[:, 2]
    else:
        v = np.log1p(np.clip(inten, 0, None))  # intensity/weight spans orders of magnitude
    v = (v - v.min()) / (np.ptp(v) + 1e-9)
    colors = plt_turbo(v)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd], window_name=f"BIEVR map: {p.name} (color={args.color})")


def plt_turbo(v):
    """Map [0,1] -> RGB without a matplotlib dependency for the 3-D path."""
    try:
        import matplotlib.cm as cm
        return cm.get_cmap("turbo")(v)[:, :3]
    except Exception:
        # simple blue->red fallback
        return np.stack([v, np.zeros_like(v), 1 - v], axis=1)


if __name__ == "__main__":
    main()
