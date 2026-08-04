# `BIEVR-LIO/` — bump-image / elevation-voxel LIO

Fork `https://github.com/QuoZer/BIEVR-LIO.git` (upstream ethz-asl), branch `main`.
Image: `bievr_lio_ros2:latest`. Chosen as the project frontend (GLIM runner-up).

> Read `../CLAUDE.md` first for the findings register and dataset layout.

---

## ⚠️ The Docker image does not contain your local changes

`docker/scripts/build_ros2.sh` builds BIEVR from a *GitHub clone of the fork*, so
anything in the working tree is invisible to `bievr_lio_ros2:latest` — a container
run silently tests upstream.

**The host builds the whole workspace natively in ~30 s** (ROS 2 Jazzy + TBB +
Ceres + yaml-cpp are all installed):

```bash
colcon build --packages-up-to bievr_lio_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Use the native path for anything C++. (`build/ install/ log/` are that colcon
workspace. They had to be deleted once after the July-2026 reorg — the CMake
caches still pointed at `~/repos/BIEVR-LIO` and every build failed.)

---

## GEODE path

Reuses GLIM's `<seq>_ros2/` bags directly (same point layout) — it does **not**
have its own converter and errors out telling you to run GLIM's.

`run_bievr_container.sh` assembles a per-run config into `bievr_out/.run_config/`
by sed-patching `config/params.yaml` so save paths point at the mounted `/out`,
then launches `process_bag.launch.py`. Env knobs: `SENSOR_CONFIG` (default
`geode`), `BIEVR_IMAGE`, `DATASET_ROOT`, `SAVE_ACCUMULATED=1` (extra large
`map_raw.pcd`), `RVIZ=1` (also re-enables the ASCII dashboard, which only makes
sense on a TTY).

Outputs `traj_lidar.txt` (TUM), `map.pcd`, `map.bumpmap` (native; read with
`scripts/load_bumpmap.py`).

**Eval gotcha unique to BIEVR:** it logs the **IMU** pose `T_W_I`, not the LiDAR
pose, so its `eval_geode.py` first composes `T_W_L = T_W_I @ T_I_L` using the
`calibration` block from the sensor config before applying the gamma→GT
transform. `geode.yaml`'s calibration is identity R + zero translation.

---

## nora path + frozen-map localization (S0 and S4 of the mapping plan)

Runs **natively, not in Docker** (see above).

```bash
./run_bievr_nora.sh                                        # S0: map a bag
MAP_LOAD=<out>/map.bumpmap ./run_bievr_nora.sh             # S4: localize, map frozen
MAP_LOAD=… RVIZ=1 ./run_bievr_nora.sh                      # watch it (implies RATE=1, MAP_STRIDE=10)
```

Default bag `~/Documents/nora_merged_150_750_mcap` — ⚠️ **deprecated slice**; the
current bag is `/media/roser/SecondTB/bags/nora_merged_full_offline_mcap` (817 s,
IMU topic `/merged_imu`), pass it as `$1`. Env knobs `OUT_DIR`,
`MAX_SCANS`, `START_OFFSET_S`, `INITIAL_POSE`, `MAP_SAVE`, `SENSOR_CONFIG`,
`IMU_TOPIC`, `BUILD=0`, `RVIZ`, `OVERRIDES`. Run-specific overrides are **merged
as YAML** into a *copy of the sensor config* (the loader merges {params, sensor}
per leaf with the sensor file winning), so `config/params.yaml` stays the single
tuning source and needs no sed-patching.

⚠️ Until 2026-08-03 those overrides were *appended as text*, and yaml-cpp keeps
the **first** of two duplicate keys — so the `topics.imu` override never applied,
which was invisible only because the slice's detected topic equalled the config
default. On the full bag it ran with a topic the bag does not carry, buffered
every scan waiting for an IMU that never came, and reached 24 GB RSS with an
empty map. The runner now merges properly and refuses a topic absent from
`metadata.yaml`. `OVERRIDES="a.b=1 c.d=2"` sets arbitrary dotted keys, which is
what `nora_loc/sweep_bievr_loc.sh` uses.

- `config/sensor_configs/nora.yaml` — `/merged_scan` + a HAP IMU topic; **both
  extrinsics identity** (one merged frame — *not* `body_calibration.json`);
  `min_range_m: 1.0` drops exactly the range-0 invalids. The IMU topic differs
  per bag (`/merged_imu` full bag, `/livox/imu_10_0_0_50` on the 150–750 slice),
  so the runner reads it from `metadata.yaml`. Finding **F04**'s g-vs-m/s² is
  handled by BIEVR itself: *"autodetected IMU as normalized (mean |acc| =
  0.955301)"*.
- `process_bag` gained `--max_scans N`, `--start_offset_s S` and `--rate R`
  (R× real time; 0 = as fast as possible). All three are no-ops when unset, are
  exposed as `process_bag.launch.py` arguments, and `rviz_config:=localization`
  picks `rviz/localization.rviz` (frozen map grey + live scan, decay 0) over the
  mapping view.
- `debug.publish_map_stride: N` publishes the loaded map once on
  `/bievr_lio/points/map` (latched, transient-local), every N-th point — without
  it a localization run looks exactly like odometry in RViz. Stride 10 on the
  nora map = 1.14 M points, which travels over DDS fine.
- **`BIEVRMap::importMap`** — mirror of `exportMap`; config keys `map.load_path`,
  `map.update` (false = frozen), `map.initial_pose: [x,y,z,qx,qy,qz,qw]`.
  Localization suppresses the three `integratePoints` sites; the LRU never runs
  (it lives inside `integratePoints`), so a frozen map cannot erode.
- **Per-scan diagnostics** (2026-08-03) — `debug.diagnostics_path: <file.csv>`
  writes one row per scan and `/bievr_lio/diagnostics`
  (`diagnostic_msgs/DiagnosticArray`) carries the same values live. Both come
  from `LsqRegistration::diagnostics()`, which exposes what the solve already
  computed and used to throw away: effective/inlier/no-correspondence counts,
  mean |point-to-surface residual| (**−1 when no points were effective**, so a
  lost lock cannot average away), Huber cost, LM iterations/λ/converged, and the
  eigenvalues of the final information matrix — `lambda_min_3`/`kappa_3` use the
  same translation-block convention as `analysis/degeneracy.py`.
  The CSV is the primary sink: offline runs replay faster than any subscriber.
  Key names match `/fastlio_diagnostics` where the quantity is the same.
  ⚠️ The `speed` column is `x_j_pred.v`, the IMU **prediction** before the
  inertial window is optimized; `analysis/loc_metrics.py` differentiates the
  trajectory instead, and that is what the reports use.
- **Bumpmap format v2 is the only format** (2026-08-03): pose + bump image +
  weights + `int32 voxel_index[3]` + `float32 bump_smoothed[rows*cols]`.
  `exportMap` writes it, `importMap` accepts *only* it — v1 support (index
  re-derived from the centroid, `bump_smoothed` recomputed via a full-image
  `maskedGaussianSmooth`) was deleted along with `scripts/check_smoothing.py`,
  which existed solely to validate that recompute. The stale v1 dump
  `~/Documents/nora_merged_150_750_bievr_out/map_v1.bumpmap` is now unreadable
  by both the C++ and the Python side; re-export from the bag if it's ever needed.
- `scripts/load_bumpmap.py` has a streaming `iter_bumpmap()` (the full loader
  needs ~1 GB on a 517 MB map). `scripts/compare_bumpmap.py A B` diffs two dumps
  **order-insensitively** (keyed on the voxel index; export order is insertion
  order, so a re-export is not guaranteed to match line for line), exit 1 on any
  difference.
- **Bump heights are per-voxel quantities and must be normalized before they are
  compared or coloured across voxels.** Each is signed along that voxel's own
  plane normal — whose direction the plane fit picks arbitrarily, so **48 % of
  adjacent near-parallel voxel pairs disagree on it** — and measured from a plane
  sitting at an arbitrary offset (11.7 % of the pooled variance on the nora crop).
  Raw, a continuous floor comes out as a blue/green/red patchwork of flat 0.5 m
  blocks. `view_map.py --color bump` demeans per voxel and flood-fills the voxel
  lattice flipping signs into agreement (48 % → 7.8 % disagreement; the remainder
  is greedy-fill conflict, no propagation over gaps). Same caveat applies to any
  statistic taken over `bump_img` across voxels.
- `scripts/view_map.py` defaults to `--mode quads` on a `.bumpmap`: every observed
  pixel drawn as a real `px_size` square in its voxel's plane at its bump height
  (the `doc/bumpimage.png` view), colour `bump|bump_raw|height|weight` (`bump` on a
  symmetric scale about 0). Needs Open3D ⇒ run
  it with `../viz_venv/bin/python`. **`--crop X,Y,Z,R` is the mode that actually
  shows the representation** — the pixels are 5 cm, so the whole 1.4 km nora drift
  is a hairline zoomed out (a 10 m ball ⇒ 3107 voxels / 230 k squares, stride 1,
  2.5 s). Whole-map draws are capped at `--max-quads` (2 M) by dropping voxels,
  stride from a sampled fill-fraction estimate (nora ⇒ stride 6, 1.9 M squares,
  3.9 s, 1.0 GB). `--mode points` is the old sampled cloud, `--mode patches` the
  2-D montage, `--save out.ply` writes the mesh instead of opening a window.

### Measured numbers (600 s / 1236.5 m slice)

`~/Documents/nora_merged_150_750_bievr_out/`, loc in `..._bievr_loc/`:

- **S0**: 5997 poses, 2.06 m/s, Δz −134 m ⇒ −10.8 % grade (finding **F03**'s real
  terrain), 62 s wall, 740 MB RSS, **173 539 exported voxels ≈ 140 per metre
  travelled**, 517 MB `.bumpmap`, 183 MB `.pcd`.
- **S4** vs the mapping trajectory: **mean 6.1 mm, max 31 mm, 0.05° attitude**,
  flat across the run. Import costs 0.7 s / 560 MB. 47 s wall (integration off).
- **Round-trip export→import→export is byte-identical** (both `.bumpmap` and
  `.pcd`) — `ankerl::unordered_dense` iterates in insertion order.
- **`bump_smoothed_` is the only image the optimizer samples** — a map loaded
  without it registers against zeros. (Historical, from when v1 was still
  loadable: recomputing it on load was exact to float32 noise, and v1 and v2 maps
  produced byte-identical localization trajectories over 6000 scans. So v2's
  +44 % file size bought round-trip exactness, not accuracy — which is why
  dropping v1 costs nothing in accuracy terms.)
- `outer_sum_` is still not serialized (only `updateNormal` uses it): an imported
  map can be localized against but **must not resume mapping**.
- Mid-map start (`--start_offset_s 300` + `map.initial_pose`) converges in ~5 s to
  11 mm mean — but `BiasInitializer` assumes a **stationary** start and returned a
  nonsense accel bias (z = 5.57) and a 10.7°-off attitude at 2 m/s. Registration
  absorbed it; a real mid-drift start needs a still moment or a prior. The
  *"Configured initial attitude disagrees…"* warning flags it.

---

## Git

The 2026-07-31 nora/localization work landed as **`9085118` "BIEVR-Loc"**
(2026-08-03) — 14 files, `bievr_map.{h,cpp}`, `pipeline.{h,cpp}`,
`config_loader.h`, `process_bag.cpp`, `run_bievr_nora.sh`, the scripts and the
RViz config. `scripts/check_smoothing.py`, deleted the same day with v1 support,
predates that commit and is gone for good rather than recoverable from history.

⚠️ **Uncommitted (2026-08-03):** the diagnostics work — `ls_optimizer.{h,cpp}`,
`pipeline.{h,cpp}`, `config_loader.h`, the three `interfaces/` publishers and
their build files, `run_bievr_nora.sh` — plus still-untracked
`config/sensor_configs/nora.yaml`, which `run_bievr_nora.sh` requires and so
should go in with them.

⚠️ **The July-29 full-bag trajectory is not reproducible, but the code is not at
fault.** `bags/nora_merged_full_offline_mcap/trajectories/bievr.txt` (and the
`BIEVR_dual_full.pcd` handed to the FAST-LIO localization project) differ from a
run today by mean 0.82 m / max 2.56 m on identical timestamps. Checked, not
assumed: `b2ba803` rebuilt in a worktree produces a trajectory **byte-identical
to HEAD's** and the same map, and `config/params.yaml` is unchanged between them,
so `9085118` did not move mapping. The July-29 run's *configuration* was never
recorded — it predates this script and its `.run_config/` snapshot. Single-knob
probes narrow it (`min_range_m 0.0` → 0.665 m, `max_range_m 100` → 0.693 m) but
do not close it. Treat those artifacts as from an unknown config, and say which
map a number came from.

The GEODE work and map export *are* committed (`b2ba803`, `656938d`).
Keep `build/ install/ log/` out of commits.
