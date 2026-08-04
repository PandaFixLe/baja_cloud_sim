#!/usr/bin/env bash
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "22.04" ]]; then
  echo "This installer targets Ubuntu 22.04." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y locales curl gnupg lsb-release software-properties-common ca-certificates
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
if ! grep -Rqs --include='*.list' --include='*.sources' \
  'packages\.ros\.org/ros2/ubuntu' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
else
  echo "Existing ROS 2 apt source detected; leaving it unchanged."
fi

if ! grep -Rqs --include='*.list' --include='*.sources' \
  'packages\.osrfoundation\.org/gazebo/ubuntu-stable' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  # Prefer the Tsinghua mirror for both the key and the apt source (the upstream
  # OSRF host is often unreachable on networks in China). Fall back to upstream.
  if sudo curl -sSL --max-time 30 https://mirrors.tuna.tsinghua.edu.cn/osrf/ubuntu-stable \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg 2>/dev/null \
     && [ -s /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    GAZEBO_DEB="https://mirrors.tuna.tsinghua.edu.cn/osrf/ubuntu-stable"
    echo "Using Tsinghua OSRF mirror for Gazebo."
  else
    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    GAZEBO_DEB="https://packages.osrfoundation.org/gazebo/ubuntu-stable"
  fi
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] ${GAZEBO_DEB} $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
else
  echo "Existing Gazebo apt source detected; leaving it unchanged."
fi

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-ackermann-msgs \
  ros-humble-ros-gzharmonic \
  gz-harmonic \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pytest \
  mesa-utils \
  ffmpeg

# Optional remote desktop components used by start_remote_rviz.sh.
sudo apt-get install -y xvfb x11vnc fluxbox novnc websockify

# Configure rosdep to use the Tsinghua mirror (default GitHub source often
# times out on networks in China). This rewrites the default sources list to
# point at the mirror and exports ROSDISTRO_INDEX_URL so the index fetch also
# uses the mirror. Harmless on networks where GitHub is reachable.
#
# NOTE: rosdep is OPTIONAL for this project — all required dependencies are
# already installed via apt above (ros-humble-* + gz-harmonic + colcon).
# We attempt to refresh the rosdep cache, but a failure (e.g. mirror hiccup,
# EOL release data) must NOT abort the install: we fall back to a plain
# `colcon build`, which succeeds because apt already provided everything.
ROSDEP_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/rosdistro"
sudo sh -c "echo 'yaml ${ROSDEP_MIRROR}/rosdep/base.yaml' > /etc/ros/rosdep/sources.list.d/20-default.list"
sudo sh -c "echo 'yaml ${ROSDEP_MIRROR}/rosdep/python.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list"
sudo sh -c "echo 'yaml ${ROSDEP_MIRROR}/rosdep/ruby.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list"
sudo sh -c "echo 'yaml ${ROSDEP_MIRROR}/rosdep/osx-homebrew.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list"
export ROSDISTRO_INDEX_URL="${ROSDEP_MIRROR}/index.yaml"
rosdep update || echo "WARN: rosdep update failed (non-fatal); continuing without it."

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/build.sh"
echo
echo "Installation complete. Run: $SCRIPT_DIR/run.sh --seed 42"
