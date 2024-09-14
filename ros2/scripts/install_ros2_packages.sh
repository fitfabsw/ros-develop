#!/usr/bin/env bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$script_dir/../../scripts/utils.sh"
source "$(readlink -f "$script_dir/../../scripts/argparse_ros.sh")"

ros_packages=(
  # basic
  "ros-$ROSDISTRO-xacro"
  "ros-$ROSDISTRO-joy-teleop"
  # tools
  "ros-$ROSDISTRO-joint-state-publisher-gui"
  "ros-$ROSDISTRO-twist-mux"
  # rqt
  "ros-$ROSDISTRO-rqt-tf-tree"
  # gazebo
  "ros-$ROSDISTRO-gazebo-plugins"
  # learning
  "ros-$ROSDISTRO-turtlesim"
  # ros2_control stack
  "ros-$ROSDISTRO-ros2-control"
  "ros-$ROSDISTRO-ros2-controllers"
  "ros-$ROSDISTRO-control-toolbox"
  "ros-$ROSDISTRO-control-msgs"
  "ros-$ROSDISTRO-gazebo-ros2-control"
  # slam/nav
  "ros-$ROSDISTRO-slam-toolbox"
  "ros-$ROSDISTRO-navigation2"
  "ros-$ROSDISTRO-nav2-bringup"
  # dds
  "ros-$ROSDISTRO-rmw-cyclonedds-cpp"
  # rmf
  # "ros-$ROSDISTRO-rmf-traffic-editor"
  # "ros-$ROSDISTRO-rmf-building-map-tools"
  # "ros-$ROSDISTRO-rmf-traffic-editor-assets"
  # "ros-$ROSDISTRO-rmf-traffic-editor-test-maps"
  # robot
  # "ros-$ROSDISTRO-turtlebot3-simulation"
)

ros_packages_up=(
  # ros2_control stack
  # "ros-$ROSDISTRO-ros2-control"
  # "ros-$ROSDISTRO-ros2-controllers"
  # "ros-$ROSDISTRO-control-toolbox"
  # "ros-$ROSDISTRO-control-msgs"
  # slam/nav
  # "ros-$ROSDISTRO-slam-toolbox"
  "ros-$ROSDISTRO-navigation2"
  # "ros-$ROSDISTRO-nav2-bringup"
  # dds
  "ros-$ROSDISTRO-rmw-cyclonedds-cpp"
)

install_ubuntu_packages "${ros_packages[@]}"
upgrade_ubuntu_packages "${ros_packages_up[@]}"
