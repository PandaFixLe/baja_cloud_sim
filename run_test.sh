#!/usr/bin/env bash
# Obstacle-free test map — identical to run.sh except no box obstacles.
# 默认关闭感知(use_boundary=false, use_obstacle=false)：Frenet 用中心线 ±half_width
# 兜底走廊、不消费障碍消息，仅用规划控制核心跑(单独验证算法用)。
# 加 --with-perception 可恢复完整感知链路(真实车道线 + 障碍)。
# Safe to run concurrently with ./run.sh (separate GZ partition & results).
set -eo pipefail

SEED=0
OBSTACLES=0  # ← no box obstacles
USE_RVIZ=true
USE_GZ_GUI=true
USE_VIDEO=true
FINISH_MODE="none"
USE_BOUNDARY=false   # 默认关车道线检测，Frenet 走 ±half_width 兜底
USE_OBSTACLE=false   # 默认关障碍检测，纯跟踪
while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed) SEED="$2"; shift 2 ;;
    --finish-mode) FINISH_MODE="$2"; shift 2 ;;
    --no-rviz) USE_RVIZ=false; shift ;;
    --headless-gazebo) USE_GZ_GUI=false; shift ;;
    --no-video) USE_VIDEO=false; shift ;;
    --with-perception) USE_BOUNDARY=true; USE_OBSTACLE=true; shift ;;
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

export GZ_PARTITION="baja_flat_${USER//[^a-zA-Z0-9_]/_}_$SEED"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/install/baja_cloud_sim/share/baja_cloud_sim:${GZ_SIM_RESOURCE_PATH:-}"
GENERATED="$SCRIPT_DIR/runtime/scenario_0"
RESULTS="$SCRIPT_DIR/results/flat_seed_$SEED"
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
    use_boundary:="$USE_BOUNDARY" \
    use_obstacle:="$USE_OBSTACLE" \
    use_rviz:="$USE_RVIZ" \
    use_gz_gui:="$USE_GZ_GUI" \
    use_video:="$USE_VIDEO" \
    video_path:="$VIDEO_PATH"

# Auto-generate plots after run
LATEST_CSV=$(ls -t "$RESULTS"/tracking_*.csv 2>/dev/null | head -1)
if [ -n "$LATEST_CSV" ]; then
  echo ""
  echo "=== Auto-generating plots ==="
  python3 "$SCRIPT_DIR/tools/plot_tracking.py" "$LATEST_CSV"
fi
