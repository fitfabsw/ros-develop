#!/usr/bin/env bash

# galactic: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
# foxy: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
# humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# iron: https://docs.ros.org/en/iron/Installation/Alternatives/Ubuntu-Development-Setup.html

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../scripts/utils.sh"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../scripts/argparse_ros.sh"

parse_args "$@"

echo "UBUNTU_CODENAME=$UBUNTU_CODENAME"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_INSTALL_TYPE=$ROS_INSTALL_TYPE"

ensure_sudo
sudo apt-get update && sudo apt-get install -y curl gnupg

echo
echo =======================
echo set locale
echo =======================
locale # check for UTF-8
# sudo sed -i '/^en_US.UTF-8 UTF-8/s/^/#/' /etc/locale.gen
sudo sed -i '/^#\s*en_US.UTF-8 UTF-8/s/^#\s*//' /etc/locale.gen
sudo apt-get install locales
sudo locale-gen
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale # verify settings

export DEBIAN_FRONTEND=noninteractive
ln -fs /usr/share/zoneinfo/Asia/Taipei /etc/localtime
dpkg-reconfigure -f noninteractive tzdata

echo
echo =============================
echo Setup Sources
echo =============================
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo
echo =============================
echo Install development tools
echo =============================
sudo apt-get update && apt-get install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  ros-dev-tools

echo
echo ====================================================================
echo Get ROS2 code
echo ====================================================================
mkdir -p ~/ros2_$ROS_DISTRO/src
cd ~/ros2_$ROS_DISTRO
vcs import --input https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO/ros2.repos src

echo
echo ====================================================================
echo Install dependencies using rosdep
echo ====================================================================
sudo apt upgrade -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

echo
echo ====================================================================
echo Build the code in the workspace
echo ====================================================================
cd ~/ros2_$ROS_DISTRO/
colcon build --symlink-install

# echo
# echo ====================================================================
# echo Sourcing the setup script
# echo ====================================================================
append_bashrc "source ~/ros2_${ROS_DISTRO}/install/setup.bash"
