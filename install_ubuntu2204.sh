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
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
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

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/build.sh"
echo
echo "Installation complete. Run: $SCRIPT_DIR/run.sh --seed 42"
