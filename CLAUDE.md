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
  **`min_range_m: 1.2`** (was 1.0, raised 2026-08-13) — 1.0 drops exactly the
  range-0 invalids but leaves each HAP registering the *other* at ±1.1135 m as
  world geometry. 1.2 removes 95.7 % of that and costs 0.19 % of the
  accumulated cloud. ⚠️ It also moves the trajectory by **0.79 m mean / 3.02 m
  max** over 1675 m — the same magnitude as the `min_range_m 0.0` (0.665 m) and
  `max_range_m 100` (0.693 m) probes below, so the two maps are **not in one
  frame** and no grid metric compares across them.
  `docs/experiments/2026-08-13-self-return-remap.md`. The IMU topic differs
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
- **`bumpmap_from_pcd` builds a `.bumpmap` from any world-frame cloud**
  (2026-08-04) — `BIEVR/tools/`, built under `option(BIEVR_BUILD_TOOLS ON)`; PCL
  and yaml-cpp link to that executable only. Takes PCD/PLY, the same
  `params.yaml`+sensor config the pipeline uses (`importMap` refuses a geometry
  mismatch to 1e-9, so it must), `--override`, `--chunk-size` (default 2 M) and
  `--stride`. Writes `map.bumpmap` + `map.pcd` into a directory laid out like a
  mapping run's, so every existing consumer works on it unmodified.
  ⚠️ `install/setup.bash` mis-resolves under zsh here — run with
  `LD_LIBRARY_PATH="$PWD/install/bievr_lio/lib:/opt/ros/jazzy/lib:$LD_LIBRARY_PATH"`.
  A converted map localizes the full nora bag at **6.8 mm mean**, but a
  round-trip's per-voxel normals agree only moderately (Jaccard 0.98, median
  2.5°, **p99 47°**) — fine at this density, don't assume it for a sparse cloud.
  Weights are uniform (no sensor origin in an accumulated cloud ⇒
  `ranges=nullptr`), so `bump_weights_`/PCD `intensity` is not on an online map's
  scale. See `docs/experiments/2026-08-04-bumpmap-from-pcd.md`. This makes
  `voxel_size_m`/`pixel_size_m` sweepable for the first time — swept 2026-08-05,
  see **F08** below.
- ⚠️ **`map.pixel_size_m` has two regimes, set by the source cloud — see F08.**
  Measure points per occupied voxel first (`nora_loc/bumpmap_fill.py`).
  **Sparse (≲25 pts/voxel, e.g. a converted GLIM map):** `0.1` beats the shipped
  `0.05` on accuracy, disk, RAM *and* speed at once (0.014 88 vs 0.015 32 m RMSE,
  111 vs 297 MiB, 273 vs 459 MB RSS), and `0.025` is a cliff — it diverges at
  `voxel_size_m` 0.25 *and* 2.0 as fill collapses to 1.7–3.1 %.
  **Dense (≳70, which is what a BIEVR mapping run produces):** the ordering
  **reverses** — `0.025` is the best cell and holds lock everywhere, `0.1` is
  22 % worse, and the cliff moves to `0.0125`. Pair a fine pixel with a finer
  `preprocess.downsample_resolution_m`: at px 0.025 that is worth −29 %, at
  px 0.1 it is +45 % *worse*. Fill does not predict which cells tip over in
  either regime. **`voxel_size_m: 0.5` needs no change**, best at every pixel
  size (tested against density only at 0.5). Map size is set by pixel size alone
  (total pixels ≈ surface / pixel²), so `pixel_size_m` is the memory knob — the
  ladder spans 27 MiB to 6.7 GiB on it. Not applied to `config/params.yaml`: it
  is baked into every existing `.bumpmap` and `importMap` would reject them all.
  `docs/experiments/2026-08-05-map-source-density.md`.
  ⚠️ **A rebuilt map is not bit-reproducible** — two builds of identical geometry
  from identical input agree to ~1.5e-7 per voxel, worth mean 0.53 mm of
  trajectory (9 µm of RMSE). A run against a *fixed* map is still deterministic.
- ⚠️ **`preprocess.informed_sampling: true` is a bad default — see F07.** It keeps
  every point in the top `informed_sample_count` voxels by bump roughness and one
  point per remaining *map* voxel (0.5 m, 5× the downsample grid), so the shipped
  count of 300 throws away most of the cloud. Uniform downsampling beats it on
  accuracy *and* wall time at matched budget, and at count 3000 the two become
  identical — the prioritization contributes nothing measurable. Use
  `informed_sampling: false` + `optimization.huber_delta: 0.05`: 16 % lower RMSE
  at 9.0× real time. `informed_sample_count` became a config key on 2026-08-04
  (it was hardcoded in `pipeline.h`); the default path is unchanged, checked
  bit-identical.
  ⚠️ **That is a nora-localization setting — do not carry it to GEODE.** Swept
  there the same day: shield5 −66 %, but **shield1 +12 %**, and both tunnels are
  inert to 1.5 mm. `informed_sample_count: 100` **loses lock on both shields**
  (89 % / 88 % through, `kappa_6` → ∞) while uniform at a *smaller* budget
  survives. The knob that generalizes is `huber_delta: 0.05` alone.
  On nora it *is* robust to `voxel_size_m`: uniform wins at all four sizes
  tested, and informed **degenerates into uniform as voxels grow** — at 2.0 m it
  retains 99.5 % of the uniform budget and the two agree to 0.06 mm, which is
  F07's `informed_sample_count: 3000` result reached from the other direction.
  What matters is the fraction of observed voxels kept at full resolution.
- ⚠️ **A geometry-mismatched map aborts via uncaught `std::runtime_error`
  (exit 134)** rather than exiting cleanly. The log names the cause first.
  `pipeline.cpp` throws, `process_bag.cpp`'s `main()` does not catch. Known, unfixed.
- **Bumpmap format v2 is the only format** (2026-08-03): pose + bump image +
  weights + `int32 voxel_index[3]` + `float32 bump_smoothed[rows*cols]`.
  `exportMap` writes it, `importMap` accepts *only* it — v1 support (index
  re-derived from the centroid, `bump_smoothed` recomputed via a full-image
  `maskedGaussianSmooth`) was deleted along with `scripts/check_smoothing.py`,
  which existed solely to validate that recompute. The stale v1 dump
  `~/Documents/nora_merged_150_750_bievr_out/map_v1.bumpmap` is now unreadable
  by both the C++ and the Python side; re-export from the bag if it's ever needed.
  ⚠️ **So are all four GEODE maps** — `<seq>/bievr_out/map.bumpmap` for tunnel1,
  tunnel2, shield1 and shield5 are v1 and raise *"Unsupported format version 1"*
  (found 2026-08-06). Every nora map on the drive is v2. Re-run the sequence to
  get a readable GEODE bumpmap.
- **Ground surfaces and roughness come out of the map with no C++ change** —
  `map_processor/terrain.py` (`roughness`, `ground`) streams the v2 file through
  `scripts/load_bumpmap.py`. Roughness is `std(bump_img)` per voxel, the one
  bump statistic invariant to both the arbitrary normal sign and the arbitrary
  plane offset, so none of the `view_map.py` normalization is needed. **Read F10
  before quoting a number**: the ranking is stable across an 8× density change
  (Spearman +0.73) but the millimetre value is not, and it must always carry the
  pixel size it was measured at. `ground` must be given the trajectory *from the
  same run as the map* — a foreign trajectory seeds 85 of 8145 poses (F05).
  `docs/experiments/2026-08-06-ground-roughness.md`.
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

### S4 resource profile — full body bag (2026-08-04, `bench/runs/body_v2_fixed/`)

**43.9 cpu-ms/scan, 757 MB peak and flat, 11.2× real time on 4.9 cores**, over
3 reps at ≤0.3 % contention. `mem_at_10pct` equals the peak: a frozen map does
not grow, which is the point of measuring it.


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
