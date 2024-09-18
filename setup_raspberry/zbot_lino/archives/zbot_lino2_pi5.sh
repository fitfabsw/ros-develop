#!/bin/bash

log_and_echo() {
  tee log.txt
}

{
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$script_dir/../../scripts/utils.sh"
source sensors.sh
script_start_time=$(date +%s)
prepare_vcs_sh="$(readlink -f "$script_dir"/../../scripts/prepare_vcs.sh)"

# declare -A elapsed_times
stage_names=()
elapsed_times=()

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
    # ROS_DISTRO="iron"
    ROS_DISTRO="humble"
    ORIGINAL_IMAGE="Raspberry Pi OS 64-bit"
    ;;
  # mantic)
  #   ROS_DISTRO="humble"
  #   ORIGINAL_IMAGE="[1ebe853ca69ce507a69f97bb70f13bc1ffcfa7a2]ubuntu-22.04.2-preinstalled-server-arm64+raspi.img.xz"
  #   ;;
  *)
    echo "Ubuntu $VERSION_CODENAME is not supported."
    exit 1
    ;;
esac

# Common setup for all supported Ubuntu versions
echo "Setting up for ROS Distro: $ROS_DISTRO"
WORKSPACE="zbotlino_ws"
WORKSPACEPATH="$HOME/$WORKSPACE"
disable_needrestart # Workaround for the needrestart issue

echo "UBUNTU_CODENAME=$VERSION_CODENAME"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "WORKSPACE=$WORKSPACE"

BASE=zbotlino2
case $BASE in
  zbotlino)
    LASER_SENSOR=rplidar
    ;;
  zbotlinoinvert | zbotlinosick1 | zbotlinosick2 | zbotlino2)
    LASER_SENSOR=nanoscan3
    ;;
  *)
    echo "Invalid robot type"
    exit 1
    ;;
esac

echo "You are installing zbot_lino2 on your robot computer."
echo
echo "===========SUMMARY============"
echo "ROBOT TYPE   : $BASE"
echo "LASER SENSOR : $LASER_SENSOR"
echo
echo "This installer will edit your ~/.bashrc."
echo "Create a zbotlino_ws on your $HOME directory."
echo "Install zbot_lino ROS2 dependencies."
echo "Install udev rules on /etc/udev/rules.d folder."
echo -n "Enter [y] to continue. "
read reply
if [[ "$reply" != "y" && "$reply" != "Y" ]]; then
  echo "Exiting now."
  exit 1
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
ROS2_WORKSPACE="ros2_$ROS_DISTRO"
stage1() {
  $script_dir/../../ros2/scripts/prepare_ros2_workspace_pi5.sh -u "$UBUNTU_CODENAME" -r "$ROS_DISTRO" -w "$ROS2_WORKSPACE"
  check_last_command || return 1
  return 0
}

"$script_dir/../../scripts/create_workspace.sh" $WORKSPACE || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
  exit
fi

stage2_description="Install LIDAR/Depth Sensor ROS2 drivers"
stage2() {
  # install_rplidar
  install_librealsense2
  check_last_command || return 1
  return 0
}

stage3_description="Install apt packages"
stage3() {
  sudo apt install -y python3-vcstool build-essential
  check_last_command || return 1
  sudo apt install -y python3-websocket # for fitrobot_lino.status.service
  check_last_command || return 1
  return 0
}

stage4_description="Install micro_ros_setup"
stage4() {
  rm -rf "$WORKSPACEPATH"/{build,install}/micro_ros_setup > /dev/null 2>&1
  rm "$WORKSPACEPATH/src/micro_ros_setup"/COLCON_IGNORE > /dev/null 2>&1
  git clone https://github.com/micro-ROS/micro_ros_setup -b humble $WORKSPACEPATH/src/micro_ros_setup
  cd "$WORKSPACEPATH"
  colcon build --packages-select micro_ros_setup
  check_last_command || return 1
  return 0
}

stage5_description="Setup micro-ROS agent"
stage5() {
  source "$WORKSPACEPATH"/install/setup.bash
  rm -rf "$WORKSPACEPATH"/{build,install}/micro_ros_msgs > /dev/null 2>&1
  cd "$WORKSPACEPATH"
  ros2 run micro_ros_setup create_agent_ws.sh
  check_last_command || return 1
  ros2 run micro_ros_setup build_agent.sh
  check_last_command || return 1
  return 0
}

stage6_description="Install rosdep dependencies"
stage6() {
  vcs_repo_path="$script_dir/zbot_linov2_$ROS_DISTRO.repos"
  "$prepare_vcs_sh" "$WORKSPACE" "$vcs_repo_path"
  check_last_command || return 1
  cd "$WORKSPACEPATH"
  vcs import src < "$vcs_repo_path"
  check_last_command || return 1
  # rosdep install --from-path src --ignore-src -y

  # temporarily ignore some packages in order to build
  rosdep install --from-path src --ignore-src -y --skip-keys "depthimage_to_laserscan librealsense2 teleop_twist_keyboard teleop_twist_joy joy_linux"

  # check_last_command || return 1
  return 0
}

stage7_description="Build zbot_lino"
stage7() {
  source "$WORKSPACEPATH"/install/setup.bash
  touch "$WORKSPACEPATH/src/zbot_lino/linorobot2/linorobot2_gazebo"/COLCON_IGNORE
  touch "$WORKSPACEPATH/src/micro_ros_setup"/COLCON_IGNORE
  touch "$WORKSPACEPATH/src/uros"/COLCON_IGNORE
  cd "$WORKSPACEPATH" && colcon build --symlink-install
  rtn=$(check_last_command)
  rm "$WORKSPACEPATH/src/micro_ros_setup"/COLCON_IGNORE > /dev/null 2>&1
  rm "$WORKSPACEPATH/src/uros"/COLCON_IGNORE > /dev/null 2>&1
  return $rtn
}

stage8_description="Use newest nav2 mppi_controllers"
stage8() {
  $script_dir/../../ros2/scripts/install_mppi_controllers.sh -r "$ROS_DISTRO" -w "$WORKSPACE"
  check_last_command || return 1
  return 0
}

stage9_description="Setup audio"
stage9() {
  $script_dir/setup_audio.sh
  check_last_command || return 1
  return 0
}

stage10_description="Setup network including additional wifi driver"
stage10() {
  $script_dir/overclock.sh # for pi4
  $script_dir/install_rtl88x2bu.sh
  check_last_command || return 1
  return 0
}

stage_general "$stage1_description" stage1

# source /opt/ros/"$ROS_DISTRO"/setup.bash
source $HOME/ros2_$ROS_DISTRO/install/setup.bash
# source $HOME/$WORKSPACE/install/setup.bash

ROS_DISTRO="$(printenv ROS_DISTRO)"
if [[ "$ROS_DISTRO" == "" || "$ROS_DISTRO" == "<unknown>" ]]; then
  echo "No ROS2 distro detected"
  echo "Try running $ source /opt/ros/<ros_distro>/setup.bash and try again."
  exit 1
fi
exit

stage_general "$stage2_description" stage2
stage_general "$stage3_description" stage3
stage_general "$stage4_description" stage4
stage_general "$stage5_description" stage5
stage_general "$stage6_description" stage6
stage_general "$stage7_description" stage7
#stage_general "$stage8_description" stage8
stage_general "$stage9_description" stage9
stage_general "$stage10_description" stage10


echo "===================================================================="
echo "Setup environment variables                                         "
echo "===================================================================="
append_bashrc "export LINOROBOT2_BASE=$BASE"
append_bashrc "export LINOROBOT2_LASER_SENSOR=$LASER_SENSOR"
append_bashrc "export LINOROBOT2_DEPTH_SENSOR=$DEPTH_SENSOR"
append_bashrc "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
append_bashrc "source ${WORKSPACEPATH}/install/setup.bash"

echo
echo "===================================================================="
echo "setup systemd services                                              "
echo "===================================================================="
sudo cp "$WORKSPACEPATH/src/fitrobot/systemd/fitrobot.lino.service" /etc/systemd/system
sudo cp "$WORKSPACEPATH/src/fitrobot/systemd/fitrobot_lino.bringup.service" /etc/systemd/system
sudo cp "$WORKSPACEPATH/src/fitrobot/systemd/fitrobot_lino.status.service" /etc/systemd/system
sudo systemctl enable fitrobot.lino.service
sudo systemctl enable fitrobot_lino.bringup.service
sudo systemctl enable fitrobot_lino.status.service

print_elapsed_summary

# better to run this script manually
# ./set_network_pi5.sh

} 2>&1 | log_and_echo
