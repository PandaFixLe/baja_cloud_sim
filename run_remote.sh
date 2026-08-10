#!/usr/bin/env bash
# Start the Baja simulator in UDP remote-control mode.
set -eo pipefail

SEED=42
OBSTACLES=5
USE_RVIZ=true
USE_GZ_GUI=true
USE_VIDEO=true
LISTEN_ADDRESS="127.0.0.1"
LISTEN_PORT=5005

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --obstacles) OBSTACLES="$2"; shift 2 ;;
    --listen-address) LISTEN_ADDRESS="$2"; shift 2 ;;
    --listen-port) LISTEN_PORT="$2"; shift 2 ;;
    --no-rviz) USE_RVIZ=false; shift ;;
    --headless-gazebo) USE_GZ_GUI=false; shift ;;
    --no-video) USE_VIDEO=false; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ./build.sh first." >&2
  exit 1
fi
source "$SCRIPT_DIR/install/setup.bash"

export GZ_PARTITION="baja_remote_${USER//[^a-zA-Z0-9_]/_}_$SEED"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/install/baja_cloud_sim/share/baja_cloud_sim:${GZ_SIM_RESOURCE_PATH:-}"

GENERATED="$SCRIPT_DIR/runtime/remote_scenario_$SEED"
RESULTS="$SCRIPT_DIR/results/remote_seed_$SEED"
mkdir -p "$GENERATED" "$RESULTS"

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
VIDEO_PATH="$RESULTS/gazebo_${RUN_TAG}.mp4"

if [[ "$USE_VIDEO" == "true" ]] && ! command -v ffmpeg >/dev/null 2>&1; then
  echo "Video recording requires ffmpeg. Install it with: sudo apt-get install -y ffmpeg" >&2
  exit 1
fi

ros2 run baja_cloud_sim generate_scenario \
  --output "$GENERATED" \
  --seed "$SEED" \
  --obstacles "$OBSTACLES"

ros2 launch baja_cloud_sim remote_simulation.launch.py \
  world_file:="$GENERATED/baja_100m.sdf" \
  scenario_file:="$GENERATED/scenario.json" \
  results_dir:="$RESULTS" \
  listen_address:="$LISTEN_ADDRESS" \
  listen_port:="$LISTEN_PORT" \
  use_rviz:="$USE_RVIZ" \
  use_gz_gui:="$USE_GZ_GUI" \
  use_video:="$USE_VIDEO" \
  video_path:="$VIDEO_PATH"

LATEST_CSV=$(ls -t "$RESULTS"/tracking_*.csv 2>/dev/null | head -1)
if [[ -n "$LATEST_CSV" ]]; then
  echo ""
  echo "=== Auto-generating plots ==="
  python3 "$SCRIPT_DIR/tools/plot_tracking.py" "$LATEST_CSV"
fi
