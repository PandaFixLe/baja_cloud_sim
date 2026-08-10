#!/usr/bin/env bash
# Path recording mode: record GPS+IMU waypoints to CSV for later Frenet planning.
set -eo pipefail

OUTPUT_FILE="recorded_path.csv"
AUTO_START=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) OUTPUT_FILE="$2"; shift 2 ;;
    --auto-start) AUTO_START=true; shift ;;
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

ros2 launch baja_cloud_sim path_record.launch.py \
  output_file:="$OUTPUT_FILE" \
  auto_start:="$AUTO_START"
