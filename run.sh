#!/usr/bin/env bash
# Keep nounset disabled while sourcing ROS 2 Humble environment hooks.
set -eo pipefail

SEED=42
OBSTACLES=5
USE_RVIZ=true
USE_GZ_GUI=true
USE_VIDEO=true
FINISH_MODE="none"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --obstacles) OBSTACLES="$2"; shift 2 ;;
    --finish-mode) FINISH_MODE="$2"; shift 2 ;;
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

# 清理可能残留的 mock_perception 进程：它是真感知到达前的占位调试工具，
# 不应与真雷达/真仿真同时运行，否则会发假障碍(如正前方 12m 的 tall)卡住车辆。
if pgrep -f "mock_perception" >/dev/null 2>&1; then
  echo "Cleaning up stale mock_perception_node processes..."
  pkill -9 -f "mock_perception" 2>/dev/null || true
  sleep 1
fi

export GZ_PARTITION="baja_${USER//[^a-zA-Z0-9_]/_}_$SEED"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/install/baja_cloud_sim/share/baja_cloud_sim:${GZ_SIM_RESOURCE_PATH:-}"

# finish_mode=line 使用直线赛道（100m 土路 + 20m 停止区），
# 其余模式使用闭环赛道。
if [[ "$FINISH_MODE" == "line" ]]; then
  GENERATED="$SCRIPT_DIR/runtime/scenario_line_$SEED"
  RESULTS="$SCRIPT_DIR/results/line_seed_$SEED"
  mkdir -p "$GENERATED" "$RESULTS"
  RUN_TAG="$(date +%Y%m%d_%H%M%S)"
  VIDEO_PATH="$RESULTS/gazebo_${RUN_TAG}.mp4"

  if [[ "$USE_VIDEO" == "true" ]] && ! command -v ffmpeg >/dev/null 2>&1; then
    echo "Video recording requires ffmpeg. Install it with: sudo apt-get install -y ffmpeg" >&2
    exit 1
  fi

  ros2 run baja_cloud_sim generate_scenario --output "$GENERATED" --seed "$SEED" --obstacles "$OBSTACLES" --runout_m 20
  ros2 launch baja_cloud_sim simulation.launch.py \
    world_file:="$GENERATED/baja_100m.sdf" \
    scenario_file:="$GENERATED/scenario.json" \
    results_dir:="$RESULTS" \
    finish_mode:="$FINISH_MODE" \
    use_rviz:="$USE_RVIZ" \
    use_gz_gui:="$USE_GZ_GUI" \
    use_video:="$USE_VIDEO" \
    video_path:="$VIDEO_PATH"
else
  GENERATED="$SCRIPT_DIR/runtime/scenario_$SEED"
  RESULTS="$SCRIPT_DIR/results/seed_$SEED"
  mkdir -p "$GENERATED" "$RESULTS"
  RUN_TAG="$(date +%Y%m%d_%H%M%S)"
  VIDEO_PATH="$RESULTS/gazebo_${RUN_TAG}.mp4"

  if [[ "$USE_VIDEO" == "true" ]] && ! command -v ffmpeg >/dev/null 2>&1; then
    echo "Video recording requires ffmpeg. Install it with: sudo apt-get install -y ffmpeg" >&2
    exit 1
  fi

  ros2 run baja_cloud_sim generate_loop_scenario --output "$GENERATED" --seed "$SEED" --obstacles "$OBSTACLES"
  ros2 launch baja_cloud_sim simulation.launch.py \
    world_file:="$GENERATED/baja_loop.sdf" \
    scenario_file:="$GENERATED/loop_scenario.json" \
    results_dir:="$RESULTS" \
    finish_mode:="$FINISH_MODE" \
    use_rviz:="$USE_RVIZ" \
    use_gz_gui:="$USE_GZ_GUI" \
    use_video:="$USE_VIDEO" \
    video_path:="$VIDEO_PATH"
fi

# Auto-generate plots after run
LATEST_CSV=$(ls -t "$RESULTS"/tracking_*.csv 2>/dev/null | head -1)
if [ -n "$LATEST_CSV" ]; then
  echo ""
  echo "=== Auto-generating plots ==="
  python3 "$SCRIPT_DIR/tools/plot_tracking.py" "$LATEST_CSV"
fi
