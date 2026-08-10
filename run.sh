#!/usr/bin/env bash
# Keep nounset disabled while sourcing ROS 2 Humble environment hooks.
set -eo pipefail

SEED=42
OBSTACLES=5
USE_RVIZ=true
USE_GZ_GUI=true
USE_VIDEO=true
FINISH_MODE="none"
# 感知开关默认不强制：留空则 launch 用 yaml(params.yaml) 的 use_boundary/use_obstacle 默认。
# 只有用户显式加 --no-boundary/--no-obstacle/--with-perception 时才在命令行覆盖。
USE_BOUNDARY=""
USE_OBSTACLE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --obstacles) OBSTACLES="$2"; shift 2 ;;
    --finish-mode) FINISH_MODE="$2"; shift 2 ;;
    --no-rviz) USE_RVIZ=false; shift ;;
    --headless-gazebo) USE_GZ_GUI=false; shift ;;
    --no-video) USE_VIDEO=false; shift ;;
    --with-perception) USE_BOUNDARY=true; USE_OBSTACLE=true; shift ;;
    --no-boundary) USE_BOUNDARY=false; shift ;;
    --no-obstacle) USE_OBSTACLE=false; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

# 仅当用户显式设置过才把 use_boundary/use_obstacle 传给 launch；
# 否则不传 → launch 回退到 yaml 默认值(尊重你在 params.yaml 的手动设置)。
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
    $LAUNCH_BOUNDARY \
    $LAUNCH_OBSTACLE \
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
    $LAUNCH_BOUNDARY \
    $LAUNCH_OBSTACLE \
    video_path:="$VIDEO_PATH"
fi

# Auto-generate plots after run
LATEST_CSV=$(ls -t "$RESULTS"/tracking_*.csv 2>/dev/null | head -1)
if [ -n "$LATEST_CSV" ]; then
  echo ""
  echo "=== Auto-generating plots ==="
  python3 "$SCRIPT_DIR/tools/plot_tracking.py" "$LATEST_CSV"
fi
