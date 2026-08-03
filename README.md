<p align="center">
  <img width=400 src="doc/bievr_final.svg">
</p>


# BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps

<p align="center">
<a href="https://patripfr.github.io/bievr-lio/"><img src="https://shieldcn.dev/badge/Project-Page-gray?size=xs" alt="Project Page" /></a>
<a href="https://arxiv.org/abs/2604.14421"><img src="https://shieldcn.dev/badge/arXiv-2604.14421-b31b1b?logo=arxiv&size=xs" alt="arXiv" /></a>
<a href="https://arxiv.org/pdf/2604.14421"><img src="https://shieldcn.dev/badge/Paper-PDF-black?size=xs" alt="Paper PDF" /></a>
<a href="LICENSE"><img src="https://shieldcn.dev/badge/License-BSD--3--Clause-green?size=xs" alt="License: BSD-3-Clause" /></a>
<a href="https://youtu.be/TsDJOdthhNk"><img src="https://shieldcn.dev/badge/YouTube-red?logo=youtube&size=xs" alt="YouTube" /></a>
</p>

<p align="center">
<a href="https://github.com/ethz-asl/BIEVR-LIO/actions/workflows/build_20_04.yaml"><img src="https://shieldcn.dev/github/ethz-asl/BIEVR-LIO/ci.svg?workflow=build_20_04.yaml&label=ROS1%20Noetic&size=xs&variant=outline&mode=light" alt="Ubuntu 20.04 + ROS Noetic Build" /></a>
<a href="https://github.com/ethz-asl/BIEVR-LIO/actions/workflows/build_22_04.yaml"><img src="https://shieldcn.dev/github/ethz-asl/BIEVR-LIO/ci.svg?workflow=build_22_04.yaml&label=ROS2%20Humble&size=xs&variant=outline&mode=light" alt="Ubuntu 22.04 + ROS Humble Build" /></a>
<a href="https://github.com/ethz-asl/BIEVR-LIO/actions/workflows/build_24_04.yaml"><img src="https://shieldcn.dev/github/ethz-asl/BIEVR-LIO/ci.svg?workflow=build_24_04.yaml&label=ROS2%20Jazzy&size=xs&variant=outline&mode=light" alt="Ubuntu 24.04 + ROS Jazzy Build" /></a>
</p>

<p align="center">
  <img width='100%' src="doc/tunnel_detail.png">
</p>

BIEVR-LIO is a robust LiDAR-Inertial Odometry framework that uses a high-resolution,
voxel-wise oriented height image map to exploit subtle geometric variations in
challenging, information-sparse environments.

### Fork changes

This fork adds map persistence and a localization mode on top of upstream's
odometry, plus the plumbing to run both on new datasets:

- **Map saving** as `.pcd` (reconstructed points) and `.bumpmap` (native voxels)
  — see [Saving a map](#saving-a-map).
- **Map loading + frozen-map localization**: `BIEVRMap::importMap`, `map.update`
  and a configurable start pose — see [Localizing in a saved map](#localizing-in-a-saved-map).
  The map can also be published to RViz so a localization run is legible.
- **A native `.bumpmap` dump format** (voxel poses, bump images, voxel index and
  smoothed image) and Python tooling to read and diff dumps — see
  [The `.bumpmap` format](#the-bumpmap-format).
- **Replay controls** for `process_bag`: `max_scans`, `start_offset_s` and
  `rate` (real-time playback) — see [Replay controls](#replay-controls).
- **New sensor configs and runners** for GEODE and for RT-Autonomy's dual-Livox
  mining truck — see [Fork datasets and runners](#fork-datasets-and-runners).

<details>
<summary><b>Abstract</b></summary>
<br>
Reliable odometry is essential for mobile robots as they increasingly enter more challenging environments, which often contain little information to constrain point cloud registration, resulting in degraded LiDAR–Inertial Odometry (LIO) accuracy or even divergence. To address this, we present BIEVR-LIO, a novel approach designed specifically to exploit subtle variations in the available geometry for improved robustness. We propose a high-resolution map representation that stores surfaces as voxel-wise oriented height images. This representation can directly be used for registration without the calculation of intermediate geometric primitives while still supporting efficient updates. Since informative geometry is often sparsely distributed in the environment, we further propose a map-informed point sampling strategy to focus registration on geometrically informative regions, improving robustness in uninformative environments while reducing computational cost compared to global high-resolution sampling. Experiments across multiple sensors, platforms, and environments demonstrate state-of-the-art performance in well-constrained scenes and substantial improvements in challenging scenarios where baseline methods diverge. Additionally, we demonstrate that the fine-grained geometry captured by BIEVR-LIO can be used for downstream tasks such as elevation mapping for robot locomotion.
</details>

# Setup

The core estimator (`bievr_lio`) is a self-contained, ROS-independent library. On
top of it we provide both a **ROS1** interface (`bievr_lio_ros`) and a **ROS2**
interface (`bievr_lio_ros2`), which live side by side under `interfaces/`.

## Installation

### Dependencies

BIEVR-LIO is intentionally light on dependencies: the core estimator only needs
**[Eigen](https://eigen.tuxfamily.org)** and **[Ceres](http://ceres-solver.org)**.


Build instructions for both ROS versions are below. Each also offers an optional
Docker image for quickly trying out the system without setting up dependencies.

<details>
<summary><b>ROS1</b></summary>
<br>

### For quick testing: Docker

If you just want to try the system out without setting up dependencies, build the
image and drop into a shell inside it:

```bash
cd docker/
./run_docker_ros1.sh -b
```

The `-b` flag builds the image. On subsequent runs you can
omit it to reuse the existing image. Your `~/data` folder is mounted to
`/home/bievr/data` inside the container so you can keep datasets outside the
image.

To open another terminal inside the running container (e.g. to launch a node
and play a bag):

```bash
docker exec -it BIEVR-LIO-ROS1 /bin/bash
```

### Build

Requires [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) and
`python3-catkin-tools` (`sudo apt install python3-catkin-tools`).

Create a catkin workspace and clone BIEVR-LIO into it:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/BIEVR-LIO.git BIEVR-LIO
```

Install the Ceres version used by BIEVR-LIO with the provided script (builds
Ceres 2.2.0 from source):

```bash
./BIEVR-LIO/docker/scripts/install_ceres.sh
```

(Optional) **Livox support.** The Livox `CustomMsg` branches are only compiled if
the corresponding driver is found in the workspace at build time. Otherwise
BIEVR-LIO builds fine without them. If you need to process Livox data, clone and
build the matching driver into `~/catkin_ws/src` *before* building BIEVR-LIO
(each driver also needs its Livox-SDK installed system-wide):

- Livox gen1 (`livox_ros_driver`, enables `BIEVR_WITH_LIVOX`):
  [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) +
  [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
- Livox gen2 (`livox_ros_driver2`, enables `BIEVR_WITH_LIVOX2`):
  [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) +
  [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

Build and source it:

```bash
cd ~/catkin_ws
catkin build bievr_lio_ros
source devel/setup.bash
```
</details>

<details>
<summary><b>ROS2</b></summary>
<br>

### For quick testing: Docker

If you just want to try the system out without setting up dependencies, build the
image and drop into a shell inside it:

```bash
cd docker/
./run_docker_ros2.sh -b
```

The `-b` flag builds the image. On subsequent runs you can
omit it to reuse the existing image. Your `~/data` folder is mounted to
`/home/bievr/data` inside the container.

To open another terminal inside the running container (e.g. to launch a node
and play a bag):

```bash
docker exec -it BIEVR-LIO-ROS2 /bin/bash
```

### Build

Requires [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) and
`python3-colcon-common-extensions`
(`sudo apt install python3-colcon-common-extensions`). The system was tested on
Jazzy, but other ROS2 distributions might also work.

Create a colcon workspace and clone BIEVR-LIO into it:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone git@github.com:ethz-asl/BIEVR-LIO.git BIEVR-LIO
```

Install the Ceres version used by BIEVR-LIO with the provided script (builds
Ceres 2.2.0 from source):

```bash
./BIEVR-LIO/docker/scripts/install_ceres.sh
```

(Optional) **Livox support.** The Livox `CustomMsg` branch is only compiled if
`livox_ros_driver2` is found in the workspace at build time. Otherwise BIEVR-LIO
builds fine without it. If you need to process Livox data, clone and build the
driver into `~/colcon_ws/src` *before* building BIEVR-LIO (it also needs its
Livox-SDK2 installed system-wide). Only gen2 exists for ROS2 (enables
`BIEVR_WITH_LIVOX`):

- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) +
  [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

Build and source it (from the workspace root, so colcon picks up both `BIEVR/`,
the core, and `interfaces/ros2`):

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to bievr_lio_ros2
source install/setup.bash
```
</details>

## Run data

BIEVR-LIO provides two entry points, available for both ROS versions:

- **`process_topics`** runs online: it subscribes to the LiDAR and IMU topics and
  processes messages as they arrive. Use it with a live sensor or alongside
  `rosbag play`.
- **`process_bag`** reads a recorded bag directly and pushes its messages through
  the pipeline as fast as they can be processed. This is the preferred choice for
  offline evaluation and reproducing results. This fork can also throttle it to
  real time for visualization — see [Replay controls](#replay-controls).

In the commands below, replace `<sensor_config>` with one of the provided configs
(see [Configuration](#configuration)) or your own. Add `rviz:=true` to bring up
the visualization.

<details>
<summary><b>ROS1</b></summary>
<br>

Process live topics:

```bash
roslaunch bievr_lio_ros process_topics.launch sensor_config:=<sensor_config>
```

Replay a rosbag:

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=<sensor_config> rosbag:=/path/to/bag.bag
```
</details>

<details>
<summary><b>ROS2</b></summary>
<br>

Process live topics:

```bash
ros2 launch bievr_lio_ros2 process_topics.launch.py sensor_config:=<sensor_config>
```

Replay a rosbag2 directory:

```bash
ros2 launch bievr_lio_ros2 process_bag.launch.py sensor_config:=<sensor_config> rosbag:=/path/to/bag_dir
```
</details>

## Configuration

The configuration is split in two files:

- **`config/params.yaml`**: Algorithm parameters (map resolution, sampling,
  optimization, IMU window, ...). These are dataset-independent and **typically do
  not need to be adjusted**: the defaults have been validated across a wide range
  of sensors, platforms, and environments.
- **`config/sensor_configs/<name>.yaml`**: Per-dataset / per-sensor settings:
  the LiDAR and IMU topic names, the LiDAR→IMU extrinsic calibration, and the
  LiDAR min/max range.

Select a sensor config at launch with `sensor_config:=<name>`, which resolves to
`config/sensor_configs/<name>.yaml` (an absolute path starting with `/` is used
verbatim, so configs may also live outside the package). Likewise `params:=<name>`
(default `params`) selects `config/<name>.yaml`.

<details>
<summary><b>Provided datasets</b></summary>
<br>

We provide ready-to-use sensor configs for the following public datasets:

| Config | Dataset |
|--------|---------|
| `enwide` | [ENWIDE](https://projects.asl.ethz.ch/datasets/enwide/) |
| `ncd` | [Newer College Dataset](https://drive.google.com/drive/u/0/folders/1uR476FzjN3PfAiCknVKtuZi3_QfVvSdA) |
| `gamma` | [GEODE](https://thisparticle.github.io/geode) |
| `mars` | [MARS-LVIG](https://mars.hku.hk/dataset.html) |
| `grandtour` | [GrandTour](https://grand-tour.leggedrobotics.com/) |
</details>

<details>
<summary><b>Running on your own data</b></summary>
<br>

To run BIEVR-LIO on a new sensor or dataset, copy one of the provided sensor
configs to `config/sensor_configs/<your_name>.yaml` and adjust:

- `topics.pointcloud` / `topics.imu` : The topic names in your data.
- `calibration` : the `T_IMU_LIDAR` extrinsic (LiDAR → IMU) rotation and
  translation for your setup.
- `lidar.min_range_m` / `lidar.max_range_m` : the usable range of your LiDAR.

The algorithm parameters in `params.yaml` can usually be left at their defaults.
</details>

# Map saving and localization (fork)

Upstream BIEVR-LIO is pure odometry: the map lives and dies with the process.
This fork can write that map to disk, load it back, and register against it with
updates disabled — i.e. localize in a previously built map. All of it is driven
from the same YAML files described above; no new command-line interface.

## Saving a map

Set a save path in the `debug` section (of `params.yaml`, or of the sensor config,
which wins on a per-leaf basis):

| Key | Meaning |
|---|---|
| `debug.map_save_path` | Writes `<path>.pcd` and `<path>.bumpmap` when the bag/stream ends. Empty = off. |
| `debug.accumulated_map_save_path` | Writes `<path>.pcd`: the union of the raw registered scans, voxel-downsampled, with real LiDAR intensity. Much larger; independent of the bump map. |
| `debug.accumulated_map_leaf_m` | Leaf size for that downsample (default `0.05`). `<= 0` keeps every point. |
| `debug.trajectory_path` | TUM trajectory (`t x y z qx qy qz qw`). |

`<path>.pcd` is a binary PCD reconstructed from the BIEVR map — one point per valid
bump-image pixel, `intensity` = that pixel's accumulated weight. `<path>.bumpmap`
is the map itself (oriented voxel poses + bump images), which is what you load
back.

## Localizing in a saved map

Add a `map.load_path` and turn updates off:

```yaml
map:
  load_path: "/path/to/mine.bumpmap"
  update: False                 # freeze: no scan is ever integrated
  initial_pose: [0, 0, 0, 0, 0, 0, 1]   # optional; [x, y, z, qx, qy, qz, qw]
  publish_map_stride: 10        # see below (lives under debug:)
```

| Key | Meaning |
|---|---|
| `map.load_path` | Native `.bumpmap` to load at startup. Empty = build a map from scratch (upstream behaviour). |
| `map.update` | `False` freezes the map: registration still runs, but nothing is integrated. Defaults to `True`. |
| `map.initial_pose` | Start pose `T_W_I` **in the loaded map's frame**, TUM order (`w` last). Absent = start at the origin with the gravity-aligned attitude from bias initialization — which is what replaying the mapping run itself wants. |
| `debug.publish_map_stride` | Publish the loaded map once on `points/map` (world frame, latched), keeping every N-th point, so RViz can show what you are localizing against. `0` = off. |

Loading is strict by design — a localization run that silently fell back to
mapping would look like a success. 

Two properties worth knowing:

- **Only `roll`/`pitch` of `initial_pose` are checked.** They are observable from
  gravity, so the pipeline warns when the configured attitude disagrees with the
  measured one by more than 5°. Yaw and position are free.
- **A loaded map cannot correctly resume mapping.** `outer_sum_` (the second
  moment used to re-estimate voxel normals) is not serialized, so `update: True`
  together with `load_path` is a debugging combination, not a lifelong-mapping
  mode.

Bias initialization still assumes the platform is **stationary** at the start.
Starting mid-run (see `start_offset_s` below) works — registration against the
frozen map pulls the estimate in within a few seconds — but the initial biases and
attitude will be poor if the platform is moving.

## Replay controls

`process_bag` gained three optional arguments, all off by default. They are available both as launch arguments and
directly on the node:

| Argument | Meaning |
|---|---|
| `max_scans:=N` | Stop after N point clouds. The bag is then closed and the map/trajectory saved through the normal path, so short smoke runs still exercise the export. |
| `start_offset_s:=S` | Drop the first S seconds of the bag. With `map.initial_pose` this starts a localization run in the middle of a map. |
| `rate:=R` | Throttle replay to R× real time (`1` = wall clock). `0` = as fast as the hardware allows. |

```bash
# watch a localization run in real time, starting 300 s into the bag
ros2 launch bievr_lio_ros2 process_bag.launch.py \
  sensor_config:=<sensor_config> rosbag:=/path/to/bag_dir \
  rate:=1 start_offset_s:=300 rviz:=true rviz_config:=localization
```

`rviz_config:=localization` opens `rviz/localization.rviz` (frozen map in grey,
the live registered scan on top, pose axes) instead of the mapping view
`rviz/config.rviz`.

## The `.bumpmap` format

A binary dump of the observed voxels: a header (magic, version, voxel size, pixel
size, voxel count) followed by one record per voxel — the two voxel poses, the
centroid and normal, the point count, the row-major `bump_img` / `bump_weights`
matrices, the integer `voxel_index` (the hash key) and `bump_smoothed`.

Storing `bump_smoothed` rather than re-deriving it on load matters because the
registration samples that image **exclusively**; a map loaded without it would
register against all zeros. Storing `voxel_index` makes the hash key explicit
instead of re-deriving it from the centroid.

Python tooling in `scripts/`:

| Script | Use |
|---|---|
| `load_bumpmap.py MAP` | Parse a dump; `load_bumpmap()` for small maps, `iter_bumpmap()` to stream large ones. |
| `compare_bumpmap.py A B` | Order-insensitive diff, keyed on voxel index (export order is hash-map insertion order, so a re-export need not match line for line). Exit 1 on any difference. |
| `view_map.py` | Quick visualization. |

## Fork datasets and runners

Additional sensor configs beyond the upstream table:

| Config | Data |
|---|---|
| `geode` | GEODE metro-tunnel sequences, device γ (Livox Avia) |
| `geode_alpha` | GEODE urban-tunnel sequences, device α |
| `nora` | RT-Autonomy mining truck, dual Livox HAP merged into one cloud (both extrinsics identity, since the merged cloud and the IMU share a frame) |

Two helper scripts drive whole runs, assembling a per-run config next to the
output so what was run stays inspectable:

- **`run_bievr_container.sh [sequence]`** — GEODE sequences in the
  `bievr_lio_ros2` Docker image.
- **`run_bievr_nora.sh [bag_dir]`** — the truck bags, **built and run natively**
  (the Docker image builds BIEVR from a GitHub clone, so it would not contain
  local changes). Mapping by default; `MAP_LOAD=<map.bumpmap>` switches it to
  localization. Other knobs: `OUT_DIR`, `MAX_SCANS`, `START_OFFSET_S`,
  `INITIAL_POSE`, `RATE`, `MAP_STRIDE`, `MAP_SAVE`, `SENSOR_CONFIG`, `IMU_TOPIC`,
  `BUILD=0`, `RVIZ=1` (which implies `RATE=1` and, in localization mode, publishing
  the map).

```bash
./run_bievr_nora.sh                                              # map a bag
MAP_LOAD=<out>/map.bumpmap RVIZ=1 ./run_bievr_nora.sh            # localize, watch it
```

# Acknowledgements
We thank the authors of [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry), [Wavemap](https://github.com/ethz-asl/wavemap) and [UGPM](https://github.com/UTS-RI/ugpm) for open-sourcing their works that served as an inspiration for us.
We used [ascii-image-converter](https://github.com/TheZoraiz/ascii-image-converter) for our ascii art.

# Citation

Please cite our work if you are using BIEVR-LIO in your research.
  ```bibtex
@article{pfreundschuh2026bievr,
  title        = {BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps},
  author       = {Pfreundschuh, Patrick and Tuna, Turcan and {Le Gentil}, Cedric and Siegwart, Roland and Cadena, Cesar and Oleynikova, Helen},
  year         = 2026,
  journal      = {Robotics: Science and Systems},
}
  ```
