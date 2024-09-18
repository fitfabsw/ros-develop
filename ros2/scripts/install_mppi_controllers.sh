#!/usr/bin/env bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$(readlink -f "$script_dir/../../scripts/argparse_ros.sh")"

# LATEST_WORKED_COMMIT="1.1.12"
#a45b151ceb3aa9edebad8a528cd672935f0c668d
# LATEST_WORKED_COMMIT="a45b151c" # 2024/3/3
LATEST_WORKED_COMMIT="1.1.16" # 2024/9/18
# LATEST_WORKED_COMMIT="1.1.15" # 2024/7/30
# LATEST_WORKED_COMMIT="3ed4c2df" # including BIG improvements in MPPI !! must have !!!

WORKSPACEPATH="$HOME/$WORKSPACE"
echo "ROSDISTRO=$ROSDISTRO"
echo "WORKSPACE=$WORKSPACE"
echo "LATEST_WORKED_COMMIT=$LATEST_WORKED_COMMIT"

"$script_dir/../../scripts/create_workspace.sh" $WORKSPACE || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
  exit
fi

# In order to build the latest mppi_controllers, we need to build navigation2 from source.
# when comping from source, trying out several commit, below is the latest results
#
# ===== mppi_controllers build procedures =====
# ref: https://github.com/ros-planning/navigation2/issues/3898#issuecomment-1776063536
#   0. Ensure the navigation2 binary are all in newest version. ie. 1.1.12
#   1. remove the nav2_mppi_controller binary package (ex: ros-humble-nav2-mppi-controller)
#      ros-humble-nav2-bringup will be removed as well
#   2. copy the nav2_mppi_controller source from navigation2 repo into workspace
#   3. colcon build the nav2_mppi_controller package
#   4. re-install ros-humble-nav2-bringup
#
# (2024/7/18)
# After nav2 binary version 1.1.15, there seems no need compile mppi manually.
# Just for simulation. Real case still crashes.
#
# a45b151c worked (2024/3/3)
# 3ed4c2 worked (11/17)
#   we should be able build any commit from now on following above procedures
# humble (57f55c) failed (10/16)
# 3b791e failed (10/13)
# 1b97e3 failed [MPPI Optimization] (10/10)
# 1.1.12 ok (10/4)
# 1.1.11 failed
# 1.1.10 ok
# 1.1.9 ok (8/4)

# ref: MPPI crashing on loading plug-ings #3767
# https://github.com/ros-planning/navigation2/issues/3767

if [[ $ROSDISTRO == "humble" ]]; then
  echo "Install mppi_controllers from source."
  #
  echo "1. remove the mppi_controllers binary package"
  sudo apt remove ros-$ROSDISTRO-nav2-mppi-controller -y
  #
  echo "2. copy the nav2_mppi_controller source from navigation2 repo into workspace"
  rm -rf /tmp/navigation2 >/dev/null 2>&1
  git clone https://github.com/ros-planning/navigation2 /tmp/navigation2 -b "$ROSDISTRO"
  cd /tmp/navigation2 && git checkout $LATEST_WORKED_COMMIT
  cp -R /tmp/navigation2/nav2_mppi_controller "$WORKSPACEPATH/src"

  echo "3. colcon build the nav2_mppi_controller package"
  cd "$HOME/$WORKSPACE"
  rm -rf build/nav2_mppi_controller install/nav2_mppi_controller >/dev/null 2>&1
  colcon build --symlink-install --packages-select nav2_mppi_controller
  #
  echo "4. re-install ros-humble-nav2-bringup"
  sudo apt install ros-$ROSDISTRO-nav2-bringup -y
fi
