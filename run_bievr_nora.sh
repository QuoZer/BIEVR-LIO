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
#   OVERRIDES      extra config, space-separated dotted assignments merged into
#                  the per-run config, e.g.
#                    OVERRIDES="optimization.huber_delta=0.2 lidar.max_range_m=60"
#                  Used by the parameter sweeps. Values are parsed as YAML.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${1:-/media/roser/SecondTB/bags/nora_merged_full_body_mcap}"
SENSOR_CONFIG="${SENSOR_CONFIG:-nora}"
MAX_SCANS="${MAX_SCANS:-0}"
MAP_LOAD="${MAP_LOAD:-}"
MAP_SAVE="${MAP_SAVE:-0}"
INITIAL_POSE="${INITIAL_POSE:-}"
OVERRIDES="${OVERRIDES:-}"
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

# Without IMU the pipeline never initializes and buffers every scan instead of
# failing - a whole-bag run then dies at tens of GB of RSS with an empty map.
# Cheaper to refuse up front than to discover it 16 GB into the mcap.
grep -qF "name: $IMU_TOPIC" "$BAG_DIR/metadata.yaml" || {
  echo "IMU topic $IMU_TOPIC is not in $BAG_DIR/metadata.yaml" >&2; exit 1; }

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
#
# The overrides are merged as YAML rather than appended as text. Appending is
# what silently dropped the IMU-topic override for as long as this script has
# existed: `topics:` is already in every sensor config, and yaml-cpp keeps the
# first of two duplicate keys. A real merge also makes OVERRIDES (below) safe
# for any key, which is what the parameter sweeps need.
mkdir -p "$OUT_DIR/.run_config"
RUN_SENSOR="$OUT_DIR/.run_config/sensor.yaml"

python3 - "$SENSOR_SRC" "$RUN_SENSOR" <<PYEOF
import sys, yaml

src, dst = sys.argv[1], sys.argv[2]
cfg = yaml.safe_load(open(src)) or {}

overrides = {
    "topics": {"imu": "$IMU_TOPIC"},
    "debug": {
        "trajectory_path": "$OUT_DIR/traj_lidar.txt",
        "diagnostics_path": "$OUT_DIR/diagnostics.csv",
        "dashboard": "$RVIZ" == "1",
        "publish_map_stride": int("$MAP_STRIDE"),
    },
}
if "$MAP_LOAD" or "$MAP_SAVE" == "1":
    overrides["debug"]["map_save_path"] = "$OUT_DIR/map"
    overrides["debug"]["accumulated_map_save_path"] = "$OUT_DIR/raw_map"
if "$MAP_LOAD":
    overrides["map"] = {"load_path": "$MAP_LOAD", "update": False}
    if "$INITIAL_POSE":
        overrides["map"]["initial_pose"] = [float(v) for v in "$INITIAL_POSE".split()]

# OVERRIDES="a.b=1 c.d=xyz" - dotted paths into the same tree, for sweeps.
for item in """$OVERRIDES""".split():
    path, _, raw = item.partition("=")
    node = overrides
    keys = path.split(".")
    for key in keys[:-1]:
        node = node.setdefault(key, {})
    node[keys[-1]] = yaml.safe_load(raw)

def merge(base, extra):
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value

merge(cfg, overrides)
with open(dst, "w") as f:
    f.write("# Generated by run_bievr_nora.sh - edits here are overwritten.\n")
    yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=False)
PYEOF

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
