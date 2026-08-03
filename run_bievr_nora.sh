#!/usr/bin/env bash
# Run BIEVR-LIO on a nora (RT-Autonomy truck, dual Livox HAP) ROS2 bag.
#
# Usage:  ./run_bievr_nora.sh [BAG_DIR]
# Default bag: ~/Documents/nora_merged_150_750_mcap   (600 s pre-merged slice)
#
# Unlike run_bievr_container.sh this runs the *locally built* workspace on the
# host, not the bievr_lio_ros2 image: the image builds BIEVR from a GitHub clone
# (docker/scripts/build_ros2.sh), so local C++ changes - importMap / frozen-map
# localization among them - would not be in it. The host has ROS 2 Jazzy plus
# TBB/Ceres/yaml-cpp, and `colcon build --packages-up-to bievr_lio_ros2` takes
# ~30 s, so the native loop is both faster and actually tests the working tree.
#
# Two modes, selected by MAP_LOAD:
#   MAP_LOAD unset  -> S0 mapping: build a map, write traj + map.pcd + map.bumpmap
#   MAP_LOAD=<file> -> S4 localization: load that frozen .bumpmap, integration
#                      disabled, write the localized trajectory only
#
# Environment overrides:
#   OUT_DIR        output directory              (default <bag>_bievr_out, or
#                                                 <bag>_bievr_loc in MAP_LOAD mode)
#   SENSOR_CONFIG  name in config/sensor_configs (default: nora)
#   IMU_TOPIC      override the auto-detected IMU topic
#   MAX_SCANS      stop after N point clouds     (0 = whole bag; smoke tests)
#   START_OFFSET_S skip the first N seconds of the bag (0 = from the start).
#                  With INITIAL_POSE this starts a localization run mid-map.
#   MAP_LOAD       path to a .bumpmap to localize against (enables S4 mode)
#   MAP_SAVE       1 = write the map out even in S4 mode. With update disabled
#                  that re-exports exactly what was loaded, which is the
#                  export -> import -> export round-trip check.
#   INITIAL_POSE   "x y z qx qy qz qw" start pose in the frozen map frame
#                  (S4 only; default = whatever bias init produces, i.e. the
#                  origin, which is correct when replaying the mapping bag)
#   BUILD          0 = skip the colcon build      (default: 1)
#   RVIZ           1 = also open RViz2. In MAP_LOAD mode this uses
#                  interfaces/ros2/rviz/localization.rviz (frozen map + live
#                  scan) and switches the map publish on; otherwise config.rviz.
#   RATE           replay speed, x real time (e.g. 1 = real time). Default 0 =
#                  as fast as the hardware allows. RVIZ=1 defaults this to 1.
#   MAP_STRIDE     publish every N-th map point on /bievr_lio/points/map for
#                  RViz (0 = off; defaults to 10 when RVIZ=1 with MAP_LOAD)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${1:-$HOME/Documents/nora_merged_150_750_mcap}"
SENSOR_CONFIG="${SENSOR_CONFIG:-nora}"
MAX_SCANS="${MAX_SCANS:-0}"
MAP_LOAD="${MAP_LOAD:-}"
MAP_SAVE="${MAP_SAVE:-0}"
INITIAL_POSE="${INITIAL_POSE:-}"
BUILD="${BUILD:-1}"
RVIZ="${RVIZ:-0}"
# Watching only makes sense in real time, and only with the map on screen - so
# RVIZ=1 implies both unless they were set explicitly.
if [ "$RVIZ" = "1" ]; then
  RATE="${RATE:-1}"
  [ -n "$MAP_LOAD" ] && MAP_STRIDE="${MAP_STRIDE:-10}"
fi
RATE="${RATE:-0}"
MAP_STRIDE="${MAP_STRIDE:-0}"

if [ -n "$MAP_LOAD" ]; then
  DEFAULT_OUT="${BAG_DIR%/}_bievr_loc"
else
  DEFAULT_OUT="${BAG_DIR%/}_bievr_out"
fi
OUT_DIR="${OUT_DIR:-$DEFAULT_OUT}"

SENSOR_SRC="$SCRIPT_DIR/config/sensor_configs/${SENSOR_CONFIG}.yaml"
PARAMS_SRC="$SCRIPT_DIR/config/params.yaml"

# ── Sanity checks ─────────────────────────────────────────────────────────────
[ -f "$BAG_DIR/metadata.yaml" ] || { echo "ROS2 bag not found: $BAG_DIR" >&2; exit 1; }
[ -f "$SENSOR_SRC" ] || { echo "Sensor config not found: $SENSOR_SRC" >&2; exit 1; }
if [ -n "$MAP_LOAD" ] && [ ! -f "$MAP_LOAD" ]; then
  echo "Map to load not found: $MAP_LOAD" >&2; exit 1
fi

# ── Which IMU topic does this bag actually carry? ─────────────────────────────
# /merged_imu on the full pre-merged bag, /livox/imu_10_0_0_50 on the 150-750
# slice. Both are in livox_hap_left_frame, so the identity extrinsic holds either
# way; only the topic name changes.
if [ -z "${IMU_TOPIC:-}" ]; then
  IMU_TOPIC="$(grep -oE '/(merged_imu|livox/imu_10_0_0_5[01])' "$BAG_DIR/metadata.yaml" | head -1 || true)"
  [ -n "$IMU_TOPIC" ] || { echo "No known IMU topic in $BAG_DIR/metadata.yaml; set IMU_TOPIC." >&2; exit 1; }
fi

# ── Build the workspace ───────────────────────────────────────────────────────
# ROS's setup scripts read unset variables; -u would abort on them.
set +u
source /opt/ros/jazzy/setup.bash
if [ "$BUILD" = "1" ]; then
  echo "[bievr] building workspace ..."
  (cd "$SCRIPT_DIR" && colcon build --packages-up-to bievr_lio_ros2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release >/dev/null)
fi
# shellcheck disable=SC1091
source "$SCRIPT_DIR/install/setup.bash"
set -u

# ── Assemble the per-run config ───────────────────────────────────────────────
# The loader merges {params, sensor} per leaf with the sensor file winning, so
# run-specific overrides go into a copy of the sensor config and params.yaml is
# used untouched (single source of truth for the algorithm tuning).
mkdir -p "$OUT_DIR/.run_config"
RUN_SENSOR="$OUT_DIR/.run_config/sensor.yaml"
cp "$SENSOR_SRC" "$RUN_SENSOR"
{
  echo ""
  echo "# ---- injected by run_bievr_nora.sh ----"
  echo "topics:"
  echo "  imu: \"$IMU_TOPIC\""
  echo "debug:"
  echo "  trajectory_path: \"$OUT_DIR/traj_lidar.txt\""
  { [ -z "$MAP_LOAD" ] || [ "$MAP_SAVE" = "1" ]; } && echo "  map_save_path: \"$OUT_DIR/map\""
  echo "  dashboard: $([ "$RVIZ" = "1" ] && echo True || echo False)"
  echo "  publish_map_stride: $MAP_STRIDE"
  if [ -n "$MAP_LOAD" ]; then
    echo "map:"
    echo "  load_path: \"$MAP_LOAD\""
    echo "  update: False"
    [ -n "$INITIAL_POSE" ] && echo "  initial_pose: [$(echo "$INITIAL_POSE" | tr ' ' ',')]"
  fi
} >> "$RUN_SENSOR"

echo "[bievr] bag      : $BAG_DIR"
echo "[bievr] imu topic: $IMU_TOPIC"
echo "[bievr] config   : $SENSOR_CONFIG -> $RUN_SENSOR"
echo "[bievr] mode     : $([ -n "$MAP_LOAD" ] && echo "localization (frozen $MAP_LOAD)" || echo mapping)"
echo "[bievr] output   : $OUT_DIR"

ARGS=(--sensor_config_file "$RUN_SENSOR" --params_file "$PARAMS_SRC" --bag "$BAG_DIR")
[ "$MAX_SCANS" != "0" ] && ARGS+=(--max_scans "$MAX_SCANS")
[ "${START_OFFSET_S:-0}" != "0" ] && ARGS+=(--start_offset_s "$START_OFFSET_S")
[ "$RATE" != "0" ] && ARGS+=(--rate "$RATE")

LOG_FILE="$OUT_DIR/run.log"
echo "[bievr] log      : $LOG_FILE"

if [ "$RVIZ" = "1" ]; then
  RVIZ_CFG="$SCRIPT_DIR/interfaces/ros2/rviz/$([ -n "$MAP_LOAD" ] && echo localization || echo config).rviz"
  echo "[bievr] rviz     : $RVIZ_CFG (replay ${RATE}x real time)"
  rviz2 -d "$RVIZ_CFG" >/dev/null 2>&1 &
  RVIZ_PID=$!
  trap 'kill $RVIZ_PID 2>/dev/null || true' EXIT
  sleep 3  # let RViz come up and subscribe before the first frames go out
fi

/usr/bin/time -v ros2 run bievr_lio_ros2 process_bag "${ARGS[@]}" 2>&1 | tee "$LOG_FILE"

echo "[done] Output in $OUT_DIR"
