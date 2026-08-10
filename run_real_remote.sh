#!/usr/bin/env bash
# Real-car remote teleop mode: UDP remote control + CAN bridge.
set -eo pipefail

USE_RVIZ=true
# 感知开关默认不强制：留空则 launch 用 yaml(real_car_params.yaml) 默认。
# 仅显式 flag 才覆盖。
USE_BOUNDARY=""
USE_OBSTACLE=""
LISTEN_ADDRESS="0.0.0.0"
LISTEN_PORT=5005
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz) USE_RVIZ=false; shift ;;
    --no-boundary) USE_BOUNDARY=false; shift ;;
    --no-obstacle) USE_OBSTACLE=false; shift ;;
    --listen-address) LISTEN_ADDRESS="$2"; shift 2 ;;
    --listen-port) LISTEN_PORT="$2"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

LAUNCH_BOUNDARY=""
LAUNCH_OBSTACLE=""
if [[ -n "$USE_BOUNDARY" ]]; then LAUNCH_BOUNDARY="use_boundary:=$USE_BOUNDARY"; fi
if [[ -n "$USE_OBSTACLE" ]]; then LAUNCH_OBSTACLE="use_obstacle:=$USE_OBSTACLE"; fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ./build.sh first." >&2
  exit 1
fi
source "$SCRIPT_DIR/install/setup.bash"

RESULTS="$SCRIPT_DIR/results_real"
mkdir -p "$RESULTS"

ros2 launch baja_cloud_sim real_car_remote.launch.py \
  use_rviz:="$USE_RVIZ" \
  $LAUNCH_BOUNDARY \
  $LAUNCH_OBSTACLE
