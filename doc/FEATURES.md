# Features

## Core odometry (upstream)

- **Bump-image voxel map**: surfaces are stored as high-resolution, voxel-wise
  oriented height images rather than raw points or intermediate geometric
  primitives, enabling direct, efficient registration and updates.
- **Map-informed point sampling**: focuses registration on geometrically
  informative regions, improving robustness in feature-sparse environments
  (e.g. tunnels) while cutting compute versus global high-resolution sampling.
- **LiDAR-inertial fusion**: tightly-coupled IMU integration and optimization
  (Ceres-based) for accurate pose estimation across sensors and platforms.
- **ROS-independent core**: `bievr_lio` only depends on Eigen and Ceres, with
  separate ROS1 (`bievr_lio_ros`) and ROS2 (`bievr_lio_ros2`) interface
  packages layered on top.
- **Two entry points** for both ROS versions:
  - `process_topics` — online processing from live/subscribed LiDAR + IMU topics.
  - `process_bag` — offline batch processing of a recorded bag, the preferred
    path for evaluation and reproducing results.
- **Optional Livox support**: `CustomMsg` support for Livox gen1
  (`livox_ros_driver`) and gen2 (`livox_ros_driver2`) drivers, compiled in
  automatically when the corresponding driver is present in the workspace.
- **Docker images** for both ROS1 and ROS2 to try the system without setting
  up dependencies locally.
- **Ready-to-use sensor configs** for public datasets: ENWIDE, Newer College
  Dataset, GEODE, MARS-LVIG, and GrandTour.
- **Split configuration**: dataset-independent algorithm parameters
  (`config/params.yaml`) separate from per-sensor/per-dataset settings
  (`config/sensor_configs/<name>.yaml`), so new sensors only need topic names,
  extrinsics, and range limits.
- **RViz visualization** via `rviz:=true` on either entry point.
- **Downstream use of the fine-grained map** (e.g. elevation mapping for
  robot locomotion), enabled by the bump-image representation.

## Fork additions

- **Map persistence**: save the built map to disk as `<path>.pcd`
  (reconstructed points, one per valid bump-image pixel with accumulated
  weight as intensity) and/or `<path>.bumpmap` (the native voxel
  representation), plus an optional accumulated-scan point cloud and TUM
  trajectory export — all controlled from `debug.*` config keys.
- **Frozen-map localization mode**: load a previously saved `.bumpmap` via
  `map.load_path`, disable further map updates (`map.update: False`), and
  register live scans against it — with strict loading (no silent fallback to
  mapping) and an optional configurable start pose.
- **Gravity-consistency check on startup pose**: warns if the configured
  `roll`/`pitch` of `initial_pose` disagrees with the measured attitude by
  more than 5°.
- **Map publishing for localization runs**: `debug.publish_map_stride`
  latches the loaded map onto `points/map` (downsampled by stride) so RViz
  can show what a localization run is registering against, using a dedicated
  `rviz/localization.rviz` view.
- **Native `.bumpmap` binary format**: a documented, versioned dump of voxel
  poses, bump images/weights, voxel index, and the smoothed image actually
  used for registration.
- **Python tooling for `.bumpmap` files** (`scripts/`):
  - `load_bumpmap.py` — parse dumps (`load_bumpmap()` for small maps,
    `iter_bumpmap()` to stream large ones).
  - `compare_bumpmap.py` — order-insensitive diff between two dumps, keyed on
    voxel index.
  - `view_map.py` — quick visualization of a dump.
- **Replay controls for `process_bag`**: `max_scans` (stop after N clouds,
  still exporting map/trajectory), `start_offset_s` (skip the start of a bag,
  useful for starting localization mid-run), and `rate` (throttle playback to
  real time or faster for live visualization).
- **Additional sensor configs and runner scripts**:
  - `geode` / `geode_alpha` — GEODE metro/urban tunnel sequences (Livox Avia).
  - `nora` — RT-Autonomy's dual-Livox HAP mining truck (merged cloud).
  - `run_bievr_container.sh` — runs GEODE sequences inside the ROS2 Docker
    image.
  - `run_bievr_nora.sh` — builds and runs natively against the truck bags,
    supporting both mapping and (via `MAP_LOAD`) localization, with knobs for
    output directory, scan limits, start offset, initial pose, playback rate,
    map stride, and RViz.
