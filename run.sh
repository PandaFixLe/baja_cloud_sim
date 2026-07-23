#!/usr/bin/env bash
# Keep nounset disabled while sourcing ROS 2 Humble environment hooks.
set -eo pipefail

SEED=42
OBSTACLES=5
USE_RVIZ=true
USE_GZ_GUI=true
NO_REGENERATE=false
OBSTACLES_CONFIG=""
SAVE_OBSTACLES_CONFIG=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --obstacles) OBSTACLES="$2"; shift 2 ;;
    --no-rviz) USE_RVIZ=false; shift ;;
    --headless-gazebo) USE_GZ_GUI=false; shift ;;
    --no-regenerate) NO_REGENERATE=true; shift ;;
    --obstacles-config) OBSTACLES_CONFIG="$2"; shift 2 ;;
    --save-obstacles-config) SAVE_OBSTACLES_CONFIG=true; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Kill any lingering Gazebo / ROS processes from previous runs to prevent
# clock-partition conflicts and "Moved backwards in time" flickering.
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz sim server" 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
sleep 1
source /opt/ros/humble/setup.bash
if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ./build.sh first." >&2
  exit 1
fi
source "$SCRIPT_DIR/install/setup.bash"

export GZ_PARTITION="baja_${USER//[^a-zA-Z0-9_]/_}_$SEED"
export GZ_IP="127.0.0.1"
# Stabilise OGRE2 / Mesa software rendering in VMware (no GPU).
export LIBGL_ALWAYS_SOFTWARE=true
export GALLIUM_DRIVER=llvmpipe
export vblank_mode=0
export mesa_glthread=true
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/install/baja_cloud_sim/share/baja_cloud_sim:${GZ_SIM_RESOURCE_PATH:-}"
GENERATED="$SCRIPT_DIR/runtime/scenario_$SEED"
RESULTS="$SCRIPT_DIR/results/seed_$SEED"
mkdir -p "$GENERATED" "$RESULTS"

if $NO_REGENERATE; then
  if [[ ! -f "$GENERATED/baja_100m.sdf" ]] || [[ ! -f "$GENERATED/scenario.json" ]]; then
    echo "Error: --no-regenerate used but $GENERATED/baja_100m.sdf or scenario.json not found." >&2
    echo "Run without --no-regenerate first to generate them." >&2
    exit 1
  fi
  echo "Skipping world generation, using existing files in $GENERATED"
else
  GEN_ARGS=("--output" "$GENERATED" "--seed" "$SEED" "--obstacles" "$OBSTACLES")
  if [[ -n "$OBSTACLES_CONFIG" ]]; then
    GEN_ARGS+=("--obstacles-config" "$OBSTACLES_CONFIG")
  fi
  if $SAVE_OBSTACLES_CONFIG; then
    GEN_ARGS+=("--save-obstacles-config")
  fi
  ros2 run baja_cloud_sim generate_scenario "${GEN_ARGS[@]}"
fi
exec ros2 launch baja_cloud_sim simulation.launch.py \
  world_file:="$GENERATED/baja_100m.sdf" \
  scenario_file:="$GENERATED/scenario.json" \
  results_dir:="$RESULTS" \
  use_rviz:="$USE_RVIZ" \
  use_gz_gui:="$USE_GZ_GUI"
