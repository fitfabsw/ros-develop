#!/bin/bash

script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
source "$script_dir/../../scripts/argparse_ros.sh"
source "$script_dir/../../scripts/utils.sh"
colcon_build_sh="$(realpath "$script_dir"/../../bin/colcon_build)"

# Detect Ubuntu version and set ROS distribution accordingly
. /etc/os-release
case $VERSION_CODENAME in
focal)
  ROS_DISTRO="galactic"
  ORIGINAL_IMAGE="[488d07354c8b92592c3c0e759b0f4730dce21dce]ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz"
  ;;
jammy)
  ROS_DISTRO="humble"
  ORIGINAL_IMAGE="[1ebe853ca69ce507a69f97bb70f13bc1ffcfa7a2]ubuntu-22.04.2-preinstalled-server-arm64+raspi.img.xz"
  ;;
bookworm)
  ROS_DISTRO="iron"
  ORIGINAL_IMAGE="Raspberry Pi OS 64-bit"
  ;;
*)
  echo "Ubuntu $VERSION_CODENAME is not supported."
  exit 1
  ;;
esac

echo "Setting up for ROS Distro: $ROS_DISTRO"
WORKSPACE="central_ws"
WORKSPACEPATH="$HOME/$WORKSPACE"
disable_needrestart # Workaround for the needrestart issue
echo "UBUNTU_CODENAME=$VERSION_CODENAME"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "WORKSPACE=$WORKSPACE"

echo ===============================================
echo Prepare workspace
echo ===============================================
"$(realpath "$script_dir"/../../scripts/create_workspace.sh)" "$WORKSPACE" || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
  exit
fi

stage_general() {
  describe_stage=$1
  action=$2
  title "$describe_stage"
  stage_start_time=$(date +%s)
  $action
  check_exit_code $? "$describe_stage"
  calculate_and_store_time $stage_start_time "$describe_stage"
}

stage1_description="Prepare ROS2 environment and workspace"
stage1() {
  $script_dir/../../ros2/scripts/prepare_ros2_workspace.sh -w "$WORKSPACE"
  check_last_command || return 1
  return 0
}

stage_general "$stage1_description" stage1

source /opt/ros/"$ROS_DISTRO"/setup.bash
ROS_DISTRO="$(printenv ROS_DISTRO)"
if [[ "$ROS_DISTRO" == "" || "$ROS_DISTRO" == "<unknown>" ]]; then
  echo "No ROS2 distro detected"
  echo "Try running $ source /opt/ros/<ros_distro>/setup.bash and try again."
  exit 1
fi

stage_central_description="stage for central download/build"
stage_central() {
  REPOS_CENTRAL="$script_dir/central.repos"
  cmd="$colcon_build_sh -w $WORKSPACE -r "$REPOS_CENTRAL" -cdf ${TOKEN:+-t$TOKEN}"
  echo "$cmd" && eval "$cmd"
  #
  PACKAGES_LINO2_NO_SYMLINK="fitrobot_interfaces"
  cd $WORKSPACEPATH
  cmd="colcon build --packages-select $PACKAGES_LINO2_NO_SYMLINK --cmake-clean-cache"
  echo "$cmd" && eval "$cmd"
  #
  source "$WORKSPACEPATH"/install/setup.bash
  IGNORE_CENTRAL="linorobot2_gazebo,micro_ros_setup,uros"
  echo "# build the rest repos that ues colcon build --symlink-install"
  cmd="$colcon_build_sh -w $WORKSPACE -r "$REPOS_LINO2" -bs --CLEAN_CACHE -i $IGNORE_CENTRAL"
  echo "$cmd" && eval "$cmd"
}

stage_general "$stage_central_description" stage_central

echo "===================================================================="
echo "Setup environment variables                                         "
echo "===================================================================="
append_bashrc "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
append_bashrc "source ${WORKSPACEPATH}/install/setup.bash"

# echo
# echo "===================================================================="
# echo "setup systemd services                                              "
# echo "===================================================================="
# sudo cp "$WORKSPACEPATH/src/fitrobot/systemd/fitrobot.lino.service" /etc/systemd/system
# sudo cp "$WORKSPACEPATH/src/fitrobot/systemd/fitrobot_lino.bringup.service" /etc/systemd/system
# sudo cp "$WORKSPACEPATH/src/fitrobot/systemd/fitrobot_lino.status.service" /etc/systemd/system
# sudo systemctl enable fitrobot.lino.service
# sudo systemctl enable fitrobot_lino.bringup.service
# sudo systemctl enable fitrobot_lino.status.service

print_elapsed_summary

# better to run this script manually
# ./set_network.sh
