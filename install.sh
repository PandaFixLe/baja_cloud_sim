#!/usr/bin/env bash
# =============================================================================
# install.sh — 发布者环境对齐脚本 (baja_cloud_sim)
# -----------------------------------------------------------------------------
# 目的：
#   不同电脑运行相同代码效果不同的根因是「环境版本与配置不一致」。本脚本
#   把发布者（PandaFixLe）验证过的环境指纹（版本号/配置）固化在文件顶部，
#   使用者 clone 本项目后直接 `sudo bash install.sh` 即可：
#     1) 校验当前机器是否与发布者环境对齐；
#     2) 对可自动处理的部分（Python 包版本、ROS/rosdep 源）自动对齐；
#     3) 对不可自动处理的部分（OS 版本、内核、Gazebo 库版本）明确告警。
#
# 注意：
#   本脚本只负责「对齐已存在环境」，不重复安装 apt 系统级依赖。
#   干净的 Ubuntu 22.04 请先跑 `sudo bash install_ubuntu2204.sh`（会装齐
#   所有 apt 依赖并调用 build.sh），再跑本脚本做精细对齐。
#
# 维护：
#   环境指纹集中声明在下方 CONFIG 区。发布者环境变化后，只需修改对应常量，
#   然后 `git add install.sh && git commit && git push`，使用者重新 clone/
#   pull 即可拿到最新对齐基准。
# =============================================================================
set -euo pipefail

# -----------------------------------------------------------------------------
# CONFIG — 发布者环境指纹（PandaFixLe，最后更新于 2026-08-05）
# 修改这里并 push 即可更新全队的环境基准。
# -----------------------------------------------------------------------------
PUBLISHER_OS_VERSION="22.04"
PUBLISHER_OS_CODENAME="jammy"
PUBLISHER_OS_PRETTY="Ubuntu 22.04.5 LTS"
PUBLISHER_KERNEL="6.8.0-136-generic"

PUBLISHER_ROS_DISTRO="humble"
PUBLISHER_ROS_DESKTOP_VERSION="0.10.0-1jammy.20260612.213429"
PUBLISHER_ROS_GZHARMONIC_VERSION="0.244.12-3jammy"

PUBLISHER_GZ_HARMONIC_VERSION="1.0.0-1~jammy"
PUBLISHER_GZ_SIM_VERSION="8.14.0-1~jammy"
PUBLISHER_GZ_TRANSPORT_VERSION="13.5.0-1~jammy"

PUBLISHER_PYTHON_VERSION="3.10.12"
PUBLISHER_PIP_VERSION="22.0.2"

# Python 包精确版本（与发布者 pip3 list 一致，用于对齐）
declare -A PUBLISHER_PIP_PKGS=(
  ["numpy"]="1.21.5"
  ["scipy"]="1.8.0"
  ["matplotlib"]="3.5.1"
  ["PyYAML"]="5.4.1"
  ["pytest"]="6.2.5"
  ["pytest-cov"]="3.0.0"
  ["empy"]="3.3.4"
  ["lark"]="1.1.1"
  ["setuptools"]="59.6.0"
)

PUBLISHER_GCC_VERSION="11.4.0"
PUBLISHER_CMAKE_VERSION="3.22.1"
PUBLISHER_GIT_VERSION="2.34.1"

# ROS apt 源与 rosdep 镜像（国内网络可用，干净环境也能跑）
PUBLISHER_ROS_APT_REPO="http://packages.ros.org/ros2/ubuntu"
PUBLISHER_ROSDEP_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/rosdistro"

# -----------------------------------------------------------------------------
# 工具函数
# -----------------------------------------------------------------------------
cprintln() { printf '\033[1;36m[install.sh]\033[0m %s\n' "$*"; }
ok()      { printf '\033[1;32m  [OK]\033[0m %s\n' "$*"; }
warn()    { printf '\033[1;33m  [WARN]\033[0m %s\n' "$*" >&2; }
err()     { printf '\033[1;31m  [FAIL]\033[0m %s\n' "$*" >&2; }

ver_ge() {  # ver_ge A B -> A >= B ?  简单的三级版本号比较
  [ "$1" = "$2" ] && return 0
  local IFS=.
  read -r -a a <<< "$1"; read -r -a b <<< "$2"
  for i in 0 1 2; do
    local av=${a[i]:-0} bv=${b[i]:-0}
    ((av > bv)) && return 0
    ((av < bv)) && return 1
  done
  return 0
}

# -----------------------------------------------------------------------------
# 0. 前置检查
# -----------------------------------------------------------------------------
cprintln "==> 环境对齐开始（发布者基准：$PUBLISHER_OS_PRETTY / ROS $PUBLISHER_ROS_DISTRO）"

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "$PUBLISHER_OS_VERSION" ]]; then
  warn "OS 版本不一致：当前 $(. /etc/os-release && echo "$PRETTY_NAME")，发布者 $PUBLISHER_OS_PRETTY"
  warn "本脚本假定 Ubuntu $PUBLISHER_OS_VERSION；其它发行版可能导致行为差异。"
else
  ok "OS 版本匹配：$PUBLISHER_OS_PRETTY"
fi

CUR_KERNEL="$(uname -r)"
if [[ "$CUR_KERNEL" != "$PUBLISHER_KERNEL" ]]; then
  warn "内核版本不一致：当前 $CUR_KERNEL，发布者 $PUBLISHER_KERNEL"
  warn "内核差异通常不影响算法结果，但可能影响 Gazebo/RViz 的 GPU 与实时性表现。"
else
  ok "内核版本匹配：$CUR_KERNEL"
fi

# -----------------------------------------------------------------------------
# 1. ROS 2 安装与版本校验
# -----------------------------------------------------------------------------
cprintln "==> 校验 ROS 2 ($PUBLISHER_ROS_DISTRO)"
if [[ ! -f /opt/ros/$PUBLISHER_ROS_DISTRO/setup.bash ]]; then
  err "未找到 /opt/ros/$PUBLISHER_ROS_DISTRO/setup.bash，请先运行 install_ubuntu2204.sh"
  exit 1
fi
ok "ROS $PUBLISHER_ROS_DISTRO 已安装"

DESKTOP_VER="$(dpkg-query -W -f='${Version}' ros-humble-desktop 2>/dev/null || echo 'none')"
if [[ "$DESKTOP_VER" != "$PUBLISHER_ROS_DESKTOP_VERSION" ]]; then
  warn "ros-humble-desktop 版本不一致：当前 $DESKTOP_VER，发布者 $PUBLISHER_ROS_DESKTOP_VERSION"
else
  ok "ros-humble-desktop 版本匹配：$DESKTOP_VER"
fi

GZ_VER="$(dpkg-query -W -f='${Version}' ros-humble-ros-gzharmonic 2>/dev/null || echo 'none')"
if [[ "$GZ_VER" != "$PUBLISHER_ROS_GZHARMONIC_VERSION" ]]; then
  warn "ros-humble-ros-gzharmonic 版本不一致：当前 $GZ_VER，发布者 $PUBLISHER_ROS_GZHARMONIC_VERSION"
else
  ok "ros-humble-ros-gzharmonic 版本匹配：$GZ_VER"
fi

# -----------------------------------------------------------------------------
# 2. Gazebo (gz) 库版本校验
# -----------------------------------------------------------------------------
cprintln "==> 校验 Gazebo Harmonic 组件"
for pkg_ver in "gz-harmonic:$PUBLISHER_GZ_HARMONIC_VERSION" \
               "gz-sim8-cli:$PUBLISHER_GZ_SIM_VERSION" \
               "gz-transport13-cli:$PUBLISHER_GZ_TRANSPORT_VERSION"; do
  pkg="${pkg_ver%%:*}"; want="${pkg_ver##*:}"
  got="$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null || echo 'none')"
  if [[ "$got" != "$want" ]]; then
    warn "$pkg 版本不一致：当前 $got，发布者 $want（Gazebo 物理引擎差异会显著影响仿真轨迹）"
  else
    ok "$pkg 版本匹配：$got"
  fi
done

# -----------------------------------------------------------------------------
# 3. Python 与编译工具链版本校验
# -----------------------------------------------------------------------------
cprintln "==> 校验 Python / 工具链版本"
CUR_PY="$(python3 --version 2>&1 | awk '{print $2}')"
[[ "$CUR_PY" == "$PUBLISHER_PYTHON_VERSION" ]] && ok "Python $CUR_PY" || warn "Python 版本不一致：当前 $CUR_PY，发布者 $PUBLISHER_PYTHON_VERSION"

CUR_GCC="$(gcc --version | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')"
[[ "$CUR_GCC" == "$PUBLISHER_GCC_VERSION" ]] && ok "gcc $CUR_GCC" || warn "gcc 版本不一致：当前 $CUR_GCC，发布者 $PUBLISHER_GCC_VERSION"

CUR_CMAKE="$(cmake --version | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')"
[[ "$CUR_CMAKE" == "$PUBLISHER_CMAKE_VERSION" ]] && ok "cmake $CUR_CMAKE" || warn "cmake 版本不一致：当前 $CUR_CMAKE，发布者 $PUBLISHER_CMAKE_VERSION"

# -----------------------------------------------------------------------------
# 4. Python 包版本对齐（可自动 pip install 到发布者版本）
# -----------------------------------------------------------------------------
cprintln "==> 对齐 Python 包版本（目标 = 发布者指纹）"
PIP_BIN="$(command -v pip3 || command -v pip || echo 'pip3')"
for pkg in "${!PUBLISHER_PIP_PKGS[@]}"; do
  want="${PUBLISHER_PIP_PKGS[$pkg]}"
  got="$($PIP_BIN show "$pkg" 2>/dev/null | awk -F': ' '/^Version:/ {print $2}')"
  if [[ -z "$got" ]]; then
    warn "$pkg 未安装（需要 $want），正在安装…"
    sudo -H "$PIP_BIN" install "$pkg==$want" || warn "安装 $pkg 失败，请手动：sudo pip3 install $pkg==$want"
  elif [[ "$got" != "$want" ]]; then
    warn "$pkg 版本不一致：当前 $got，发布者 $want，正在对齐…"
    sudo -H "$PIP_BIN" install "$pkg==$want" && ok "$pkg -> $want" || warn "对齐 $pkg 失败，请手动 install"
  else
    ok "$pkg==$want"
  fi
done

# -----------------------------------------------------------------------------
# 5. ROS apt 源与 rosdep 镜像对齐（保证依赖解析稳定，国内可用）
# -----------------------------------------------------------------------------
cprintln "==> 对齐 ROS apt 源与 rosdep 镜像"
if ! grep -Rqs --include='*.list' --include='*.sources' \
  "$(echo "$PUBLISHER_ROS_APT_REPO" | sed 's#/#\\/#g')" \
  /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  warn "ROS apt 源未指向 $PUBLISHER_ROS_APT_REPO，可能导致拉到不同补丁版本。建议检查 /etc/apt/sources.list.d/ros2.list"
else
  ok "ROS apt 源匹配：$PUBLISHER_ROS_APT_REPO"
fi

if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] && \
   grep -qs "$PUBLISHER_ROSDEP_MIRROR" /etc/ros/rosdep/sources.list.d/20-default.list; then
  ok "rosdep 已使用镜像 $PUBLISHER_ROSDEP_MIRROR"
else
  warn "rosdep 未使用发布者镜像 $PUBLISHER_ROSDEP_MIRROR，依赖解析可能不稳定或超时。"
  warn "对齐命令："
  warn "  sudo sh -c \"echo 'yaml ${PUBLISHER_ROSDEP_MIRROR}/rosdep/base.yaml' > /etc/ros/rosdep/sources.list.d/20-default.list\""
  warn "  sudo sh -c \"echo 'yaml ${PUBLISHER_ROSDEP_MIRROR}/rosdep/python.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list\""
  warn "  export ROSDISTRO_INDEX_URL=${PUBLISHER_ROSDEP_MIRROR}/index.yaml && rosdep update || true"
fi

# -----------------------------------------------------------------------------
# 6. 汇总
# -----------------------------------------------------------------------------
cprintln "==> 环境对齐完成"
cprintln "若上述 FAIL/WARN 项已处理（或确认不影响你的结果），即可构建运行："
cprintln "  ./build.sh"
cprintln "  ./run.sh --seed 42"
cprintln "提示：目标确定性还依赖固定随机种子（run.sh --seed），请保持与发布者一致。"
