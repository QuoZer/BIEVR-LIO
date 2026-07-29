#!/usr/bin/env bash
# Run BIEVR-LIO on a GEODE sequence via process_bag (offline, no ROS network).
#
# Usage:  ./run_bievr_container.sh [tunnel1|tunnel2|shield1|shield5|urban_tunnel2]
# Default sequence: tunnel1
#
# Mirrors the GLIM helper (run_glim_container.sh): it reuses the ROS2 bag that
# lives at <seq_dir>/<seq>_ros2 (produced by GLIM's convert_geode_bag.py for the
# metro_tunnel/gamma sequences; for urban_tunnel2/alpha the bag is prepared
# upstream by the same convention, already carrying /velodyne_points +
# /imu/data) and runs process_bag inside the bievr_lio_ros2 Docker image,
# processing the bag as fast as the hardware allows and writing results to
# <seq_dir>/bievr_out/:
#
#   traj_lidar.txt   estimated LiDAR trajectory (TUM: t x y z qx qy qz qw)
#   map.pcd          final bump-image voxel map as a point cloud
#   map.bumpmap      final map in native bumpmap format (scripts/load_bumpmap.py)
#
# Environment overrides:
#   SENSOR_CONFIG      sensor config name in config/sensor_configs/ (default: geode)
#   BIEVR_IMAGE        Docker image to run                          (default: bievr_lio_ros2)
#   DATASET_ROOT       GEODE dataset root (default depends on sequence: metro_tunnel
#                      sequences default to .../geode/metro_tunnel, urban_tunnel2
#                      defaults to .../geode/urban_tunnel)
#   SAVE_ACCUMULATED   1 = also dump the raw accumulated cloud (map_raw.pcd, large)
#   RVIZ               1 = also open RViz2 (needs an X display)

set -euo pipefail

SEQUENCE="${1:-tunnel1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "$SEQUENCE" in
  tunnel1|tunnel2|shield1|shield5)
    DEFAULT_SENSOR_CONFIG="geode"
    DEFAULT_DATASET_ROOT="/home/roser/Documents/Datasets/geode/metro_tunnel"
    ;;
  urban_tunnel2)
    DEFAULT_SENSOR_CONFIG="geode_alpha"
    DEFAULT_DATASET_ROOT="/home/roser/Documents/Datasets/geode/urban_tunnel"
    ;;
  *)
    echo "Unknown sequence '$SEQUENCE'. Choose: tunnel1 tunnel2 shield1 shield5 urban_tunnel2" >&2
    exit 1
    ;;
esac

SENSOR_CONFIG="${SENSOR_CONFIG:-$DEFAULT_SENSOR_CONFIG}"
BIEVR_IMAGE="${BIEVR_IMAGE:-bievr_lio_ros2}"
DATASET_ROOT="${DATASET_ROOT:-$DEFAULT_DATASET_ROOT}"
SAVE_ACCUMULATED="${SAVE_ACCUMULATED:-0}"
RVIZ="${RVIZ:-0}"

SEQ_DIR="$DATASET_ROOT/$SEQUENCE"
ROS2_BAG="$SEQ_DIR/${SEQUENCE}_ros2"
BIEVR_OUT="$SEQ_DIR/bievr_out"

SENSOR_SRC="$SCRIPT_DIR/config/sensor_configs/${SENSOR_CONFIG}.yaml"
PARAMS_SRC="$SCRIPT_DIR/config/params.yaml"
ASCII_SRC="$SCRIPT_DIR/config/bievr_ascii.txt"

# ── Sanity checks ─────────────────────────────────────────────────────────────
[ -f "$ROS2_BAG/metadata.yaml" ] || {
  echo "ROS2 bag not found: $ROS2_BAG" >&2
  echo "Convert the ROS1 bag first (e.g. GLIM's convert_geode_bag.py)." >&2
  exit 1
}
[ -f "$SENSOR_SRC" ] || { echo "Sensor config not found: $SENSOR_SRC" >&2; exit 1; }

# ── Assemble a per-run config (kept next to the output for inspection) ─────────
# process_bag takes plain YAML file paths; we clone params.yaml and point the
# save paths at the mounted /out directory. Container-side paths are used because
# the config is read inside the container.
RUN_CFG="$BIEVR_OUT/.run_config"
mkdir -p "$RUN_CFG"
cp "$SENSOR_SRC" "$RUN_CFG/sensor.yaml"
[ -f "$ASCII_SRC" ] && cp "$ASCII_SRC" "$RUN_CFG/bievr_ascii.txt"

# Build the debug replacement block (2-space indent, under `debug:`).
save_block='\1trajectory_path: "/out/traj_lidar.txt"\n\1map_save_path: "/out/map"'
if [ "$SAVE_ACCUMULATED" = "1" ]; then
  save_block="$save_block"'\n\1accumulated_map_save_path: "/out/map_raw"'
fi
# The live dashboard redraws in place with cursor escapes; only useful on a TTY,
# so keep it for the interactive RViz path and disable it for headless batch.
dashboard_val=$([ "$RVIZ" = "1" ] && echo True || echo False)
sed -e "s|^\(\s*\)trajectory_path:.*|$save_block|" \
    -e 's|^\(\s*\)dashboard_ascii_path:.*|\1dashboard_ascii_path: "/out/.run_config/bievr_ascii.txt"|' \
    -e "s|^\(\s*\)dashboard:.*|\1dashboard: $dashboard_val|" \
    "$PARAMS_SRC" > "$RUN_CFG/params.yaml"

# ── Run process_bag inside the container ──────────────────────────────────────
echo "[bievr] sequence : $SEQUENCE"
echo "[bievr] bag      : $ROS2_BAG"
echo "[bievr] config   : $SENSOR_CONFIG"
echo "[bievr] output   : $BIEVR_OUT"

# Machine is shared with other agents/containers; cap CPUs so others fit.
DOCKER_CPUS="${DOCKER_CPUS:-7}"

DOCKER_ARGS=(--rm --cpus="$DOCKER_CPUS" -v "$ROS2_BAG":/data:ro -v "$BIEVR_OUT":/out)
if [ "$RVIZ" = "1" ]; then
  xhost +local:docker 2>/dev/null || true
  DOCKER_ARGS+=(-it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix)
fi

LOG_FILE="$BIEVR_OUT/run.log"
echo "[bievr] log      : $LOG_FILE"

docker run "${DOCKER_ARGS[@]}" "$BIEVR_IMAGE" bash -c "
  source /opt/ros/jazzy/setup.bash
  source ~/colcon_ws/install/setup.bash
  ros2 launch bievr_lio_ros2 process_bag.launch.py \
    sensor_config:=/out/.run_config/sensor.yaml \
    params:=/out/.run_config/params.yaml \
    rosbag:=/data \
    rviz:=$([ "$RVIZ" = "1" ] && echo true || echo false)
" 2>&1 | tee "$LOG_FILE"

echo "[done] Output in $BIEVR_OUT"
