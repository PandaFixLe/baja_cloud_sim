#!/usr/bin/env bash
# =============================================================================
# env_restore.sh — 虚拟机 -> 双系统 Ubuntu 22.04 环境对照 / 补齐脚本
# -----------------------------------------------------------------------------
# 用途：
#   你在虚拟机（发布者环境）里跑了一遍环境指纹抓取，本脚本把「发布者环境
#   快照」固化在下方 SNAPSHOT 区，并提供一个 diff/补齐动作，让双系统真机
#   上的 Ubuntu 22.04 能一键对照并自动补齐缺失项。
#
# 重要前提（务必先看 README 段）：
#   1) 真机已装好 Ubuntu 22.04 + 闭源/开源 NVIDIA 驱动（GPU 走真驱动，
#      Gazebo 不再需要虚拟机那套 --render-engine ogre 的 workaround）。
#   2) 已先跑过：
#        sudo bash install_ubuntu2204.sh   # 装 apt 系统级依赖 + 首次 build
#        sudo bash install.sh              # 精细对齐 Python 包/ROS 源指纹
#      本脚本是「对照清单 + 收尾补齐」，不是替代上面两个脚本。
#   3) 代码已拷到真机（src/ 整包 + .git，或 git clone），并处于项目根目录。
#
# 用法：
#   bash env_restore.sh            # 仅打印对照报告（diff 模式，不修改任何东西）
#   bash env_restore.sh --apply    # 在报告基础上，尝试自动补齐缺失项
#
# 维护：
#   SNAPSHOT 区是发布者在虚拟机抓取的快照（最后更新 2026-08-08）。
#   真机若想更新基准，改这里即可。
# =============================================================================
set -uo pipefail

APPLY=false
[[ "${1:-}" == "--apply" ]] && APPLY=true

# -----------------------------------------------------------------------------
# SNAPSHOT — 发布者（虚拟机）环境快照
# -----------------------------------------------------------------------------
SNAP_OS_PRETTY="Ubuntu 22.04.5 LTS"
SNAP_OS_VERSION="22.04"
SNAP_OS_CODENAME="jammy"
SNAP_KERNEL="6.8.0-136-generic"

SNAP_ROS_DISTRO="humble"
SNAP_PYTHON="3.10.12"
SNAP_GCC="11.4.0"
SNAP_CMAKE="3.22.1"
SNAP_GIT="2.34.1"

# Python 关键包（与 install.sh 指纹一致；仅列项目真正用到的，非全量）
declare -A SNAP_PIP=(
  ["numpy"]="1.21.5"
  ["scipy"]="1.8.0"
  ["matplotlib"]="3.5.1"
  ["PyYAML"]="5.4.1"
  ["pytest"]="6.2.5"
  ["pytest-cov"]="3.0.0"
  ["transforms3d"]="0.3.1"
  ["setuptools"]="59.6.0"
)

# apt 源指纹（真机应一致）
SNAP_ROS_APT_REPO="http://packages.ros.org/ros2/ubuntu jammy main"
SNAP_GAZEBO_APT_REPO="https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main"
SNAP_ROSDEP_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/rosdistro"

# ROS 环境加载（发布者 .bashrc 里没有 source 行！必须手动补到真机）
SNAP_REQUIRED_BASHRC_LINES=(
  "source /opt/ros/humble/setup.bash"
)

# -----------------------------------------------------------------------------
# 工具函数
# -----------------------------------------------------------------------------
cprintln() { printf '\033[1;36m[env_restore]\033[0m %s\n' "$*"; }
ok()      { printf '\033[1;32m  [OK]\033[0m   %s\n' "$*"; }
warn()    { printf '\033[1;33m  [MISS]\033[0m %s\n' "$*"; }
action()  { printf '\033[1;35m  [FIX]\033[0m  %s\n' "$*"; }

MISS_COUNT=0
note_missing() { warn "$*"; MISS_COUNT=$((MISS_COUNT+1)); }

# -----------------------------------------------------------------------------
cprintln "==> 环境对照报告（发布者基准：$SNAP_OS_PRETTY / ROS $SNAP_ROS_DISTRO）"
cprintln "    mode: $([ "$APPLY" = true ] && echo 'apply (自动补齐)' || echo 'report (仅报告)')"
echo

# 1) OS / 内核
CUR_OS="$(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
[[ "$CUR_OS" == "$SNAP_OS_PRETTY" ]] && ok "OS: $CUR_OS" || note_missing "OS 不一致：当前 '$CUR_OS'，基准 '$SNAP_OS_PRETTY'"
CUR_KERNEL="$(uname -r)"
[[ "$CUR_KERNEL" == "$SNAP_KERNEL" ]] && ok "kernel: $CUR_KERNEL" || note_missing "kernel 不一致：当前 '$CUR_KERNEL'，基准 '$SNAP_KERNEL'（通常不影响算法，但影响 Gazebo GPU 行为）"

# 2) 工具链
chk_ver() {  # chk_ver 名称 当前 基准
  [[ "$2" == "$3" ]] && ok "$1: $2" || note_missing "$1 不一致：当前 '$2'，基准 '$3'"
}
chk_ver "python3" "$(python3 --version 2>&1 | awk '{print $2}')" "$SNAP_PYTHON"
chk_ver "gcc"      "$(gcc --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)" "$SNAP_GCC"
chk_ver "cmake"    "$(cmake --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)" "$SNAP_CMAKE"
chk_ver "git"      "$(git --version 2>/dev/null | awk '{print $3}')" "$SNAP_GIT"

# 3) ROS 安装
if [[ -f /opt/ros/$SNAP_ROS_DISTRO/setup.bash ]]; then
  ok "ROS $SNAP_ROS_DISTRO 已安装 (/opt/ros/$SNAP_ROS_DISTRO/setup.bash)"
else
  note_missing "未找到 /opt/ros/$SNAP_ROS_DISTRO/setup.bash —— 请先跑 install_ubuntu2204.sh"
fi

# 4) Python 包对照
echo
cprintln "==> Python 包对照"
PIP_BIN="$(command -v pip3 || command -v pip || echo 'pip3')"
for pkg in "${!SNAP_PIP[@]}"; do
  want="${SNAP_PIP[$pkg]}"
  got="$($PIP_BIN show "$pkg" 2>/dev/null | awk -F': ' '/^Version:/ {print $2}')"
  if [[ -z "$got" ]]; then
    note_missing "$pkg 未安装（基准 $want）"
    if $APPLY; then action "安装: sudo -H $PIP_BIN install $pkg==$want"; sudo -H "$PIP_BIN" install "$pkg==$want" || cprintln "  安装失败，请手动执行上面的命令"; fi
  elif [[ "$got" != "$want" ]]; then
    note_missing "$pkg 版本不一致：当前 '$got'，基准 '$want'"
    if $APPLY; then action "对齐: sudo -H $PIP_BIN install $pkg==$want"; sudo -H "$PIP_BIN" install "$pkg==$want" || cprintln "  对齐失败，请手动执行"; fi
  else
    ok "$pkg==$want"
  fi
done

# 5) apt 源对照
echo
cprintln "==> apt 源对照"
if grep -Rqs --include='*.list' --include='*.sources' "$(echo "$SNAP_ROS_APT_REPO" | sed 's#/#\\/#g')" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  ok "ROS apt 源匹配"
else
  note_missing "ROS apt 源未指向 '$SNAP_ROS_APT_REPO'，请检查 /etc/apt/sources.list.d/ros2.list"
fi
if grep -Rqs --include='*.list' --include='*.sources' "$(echo "$SNAP_GAZEBO_APT_REPO" | sed 's#/#\\/#g')" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  ok "Gazebo apt 源匹配"
else
  note_missing "Gazebo apt 源未指向 '$SNAP_GAZEBO_APT_REPO'，请检查 /etc/apt/sources.list.d/gazebo-stable.list"
fi

# 6) rosdep 镜像对照
if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] && grep -qs "$SNAP_ROSDEP_MIRROR" /etc/ros/rosdep/sources.list.d/20-default.list \
   || [[ -f "$HOME/.ros/sources.list.d/20-default.list" ]] && grep -qs "$SNAP_ROSDEP_MIRROR" "$HOME/.ros/sources.list.d/20-default.list"; then
  ok "rosdep 已使用镜像 $SNAP_ROSDEP_MIRROR"
else
  note_missing "rosdep 未使用镜像 $SNAP_ROSDEP_MIRROR"
  if $APPLY; then
    action "写入 rosdep 镜像并 update"
    sudo sh -c "echo 'yaml ${SNAP_ROSDEP_MIRROR}/rosdep/base.yaml' > /etc/ros/rosdep/sources.list.d/20-default.list"
    sudo sh -c "echo 'yaml ${SNAP_ROSDEP_MIRROR}/rosdep/python.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list"
    sudo sh -c "echo 'yaml ${SNAP_ROSDEP_MIRROR}/rosdep/ruby.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list"
    export ROSDISTRO_INDEX_URL="${SNAP_ROSDEP_MIRROR}/index.yaml"
    sudo rosdep update || cprintln "  rosdep update 失败（非致命，apt 已装齐依赖）"
  fi
fi

# 7) .bashrc 中 ROS 环境加载（关键！发布者 .bashrc 漏了 source）
echo
cprintln "==> shell 环境加载对照（.bashrc）"
BASHRC="$HOME/.bashrc"
for line in "${SNAP_REQUIRED_BASHRC_LINES[@]}"; do
  if grep -qsF "$line" "$BASHRC" 2>/dev/null; then
    ok "已存在: $line"
  else
    note_missing ".bashrc 缺少: $line（新开终端会找不到 ros2 命令！）"
    if $APPLY; then
      action "追加到 $BASHRC: $line"
      printf '\n# >>> baja_cloud_sim ROS env >>>\n%s\n# <<< baja_cloud_sim ROS env <<<\n' "$line" >> "$BASHRC"
      cprintln "  已追加，请执行: source ~/.bashrc"
    fi
  fi
done

# -----------------------------------------------------------------------------
echo
cprintln "==> 对照完成：$MISS_COUNT 项缺失/不一致"
if [[ "$MISS_COUNT" -eq 0 ]]; then
  cprintln "环境已与发布者基准一致。可构建运行："
  cprintln "  ./build.sh            # 普通用户，勿用 sudo"
  cprintln "  ./run.sh --seed 42"
else
  cprintln "存在缺失项。report 模式仅查看；运行 'bash env_restore.sh --apply' 可自动补齐（部分项需 sudo）。"
  cprintln "注意：真机 GPU 走真实驱动，虚拟机 --render-engine ogre workaround 通常不需要了。"
fi
