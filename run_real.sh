#!/usr/bin/env bash
# Real-car autonomous mode: LQR path following + Frenet planning + CAN bridge.
set -eo pipefail

CSV_FILE="recorded_path.csv"
USE_RVIZ=true
USE_BOUNDARY=true
USE_OBSTACLE=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --csv) CSV_FILE="$2"; shift 2 ;;
    --no-rviz) USE_RVIZ=false; shift ;;
    --no-boundary) USE_BOUNDARY=false; shift ;;
    --no-obstacle) USE_OBSTACLE=false; shift ;;
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

ros2 launch baja_cloud_sim real_car.launch.py \
  csv_file:="$CSV_FILE" \
  use_rviz:="$USE_RVIZ" \
  use_boundary:="$USE_BOUNDARY" \
  use_obstacle:="$USE_OBSTACLE"
