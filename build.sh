#!/usr/bin/env bash
# ROS 2 Humble setup scripts read some optional variables before testing them,
# so nounset (-u) is intentionally not enabled here.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$SCRIPT_DIR"
# Harmonic-on-Humble packages come from the OSRF repository and intentionally
# conflict with Humble's default Fortress ros_gz debs. Skip these two rosdep
# keys because install_ubuntu2204.sh already installed their Harmonic variants.
#
# rosdep is best-effort: all runtime deps are already provided by apt, so if
# rosdep fails (mirror/network/EOL-data issues) we skip it and build directly.
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys "ros_gz_sim ros_gz_bridge" \
  || echo "WARN: rosdep install skipped (non-fatal); apt already provides deps."
colcon build --event-handlers console_direct+
echo "Built workspace at $SCRIPT_DIR/install"
