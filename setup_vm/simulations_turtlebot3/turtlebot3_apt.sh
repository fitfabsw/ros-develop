#!/bin/bash
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
install_script="$(realpath "$script_dir/../../scripts/install_ros_packages.sh")"
# shellcheck source=../../scripts/argparse.sh
source "$(realpath "$script_dir/../../scripts/utils.sh")"

ROS_DISTRO="${ROS_DISTRO-humble}"

echo
echo ====================================================================
echo Install turtlebot3 ros2 packages
echo ====================================================================
ros_packages=(
  "turtlebot3"
  "turtlebot3-msgs"
  "turtlebot3-simulations"
  "turtlebot3-navigation2"
)
$install_script "$ROS_DISTRO" "${ros_packages[@]}"
