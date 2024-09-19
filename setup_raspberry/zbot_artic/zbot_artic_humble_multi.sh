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
# mantic)
#   ROS_DISTRO="humble"
#   ORIGINAL_IMAGE="[1ebe853ca69ce507a69f97bb70f13bc1ffcfa7a2]ubuntu-22.04.2-preinstalled-server-arm64+raspi.img.xz"
#   ;;
*)
  echo "Ubuntu $VERSION_CODENAME is not supported."
  exit 1
  ;;
esac

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

echo ===============================================
echo Prepare ROS2 workspace
echo ===============================================
$script_dir/../../ros2/scripts/prepare_ros2_workspace.sh -w "$WORKSPACE"

# Added temporarily for testing
../../ros2/scripts/install_mppi_controllers.sh -r $ROS_DISTRO -w $WORKSPACE

echo
echo ====================================================================
echo Create udev rules
echo ====================================================================
./create_udev.sh

echo ===============================================
echo Install packages from apt
echo ===============================================
../../scripts/install_from_apt.sh $WORKSPACE $ROS_DISTRO "ros_packages.sh" "false"

# Temporarily. Needed by usb_cam. Need to be handled more systematically
sudo apt install python3-pip python3-websocket raspi-config -y
../../scripts/install_python_packages.sh pydantic

echo
echo ===============================================
echo Build/Install robots packages from source
echo ===============================================
# source "/opt/ros/${ROS_DISTRO}/setup.bash" && ../../scripts/install_from_source.sh -w $WORKSPACE -v "$script_dir/zbot_artic_$ROS_DISTRO.repos"
REPOS_ARTIC="$script_dir/zbot_artic_"$ROS_DISTRO"_multi.repos"
IGNORE_ARTIC="gazebo_ros2_control_demos,gazebo_ros2_control"
cmd="$colcon_build_sh -w $WORKSPACE -r "$REPOS_ARTIC" -cdfbs --CLEAN_CACHE ${TOKEN:+-t$TOKEN} -i $IGNORE_ARTIC"
echo "$cmd" && eval "$cmd"

echo ======== Env Variables ========
append_bashrc "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
append_bashrc "export ROBOT_TYPE=artic"
echo
echo "Do you want to add sourcing of $WORKSPACE on your ~/.bashrc?"
echo -n "Yes [y] or No [n]: "
read reply
WORKSPACEPATH="$HOME/$WORKSPACE"
if [[ "$reply" == "y" || "$reply" == "Y" ]]; then
  append_bashrc "source ${WORKSPACEPATH}/install/setup.bash"
else
  echo
  echo "Remember to run $ source ${WORKSPACEPATH}/install/setup.bash every time you open a terminal."
fi
