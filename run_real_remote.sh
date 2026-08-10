#!/usr/bin/env bash
# Real-car remote teleop mode: UDP remote control + CAN bridge.
set -eo pipefail

USE_RVIZ=true
USE_PERCEPTION=true
LISTEN_ADDRESS="0.0.0.0"
LISTEN_PORT=5005
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz) USE_RVIZ=false; shift ;;
    --no-perception) USE_PERCEPTION=false; shift ;;
    --listen-address) LISTEN_ADDRESS="$2"; shift 2 ;;
    --listen-port) LISTEN_PORT="$2"; shift 2 ;;
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

RESULTS="$SCRIPT_DIR/results_real"
mkdir -p "$RESULTS"

ros2 launch baja_cloud_sim real_car_remote.launch.py \
  use_rviz:="$USE_RVIZ" \
  use_perception:="$USE_PERCEPTION"
