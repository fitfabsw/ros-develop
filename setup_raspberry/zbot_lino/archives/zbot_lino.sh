#!/bin/bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$script_dir/../../scripts/utils.sh"
prepare_vcs_sh="$(readlink -f $script_dir/../../scripts/prepare_vcs.sh)"
UBUNTU_CODENAME=$(cat /etc/os-release | grep VERSION_CODENAME | cut -d"=" -f2)

if [[ "$UBUNTU_CODENAME" == "focal" ]]; then
  echo "Ubuntu 20.04 detected. Set ROS_DISTRO to galactic."
  ROS_DISTRO="galactic"
  ORIGINAL_IMAGE="[488d07354c8b92592c3c0e759b0f4730dce21dce]ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz"
  IMAGE_DOWNLOAD_SITE=
elif [[ "$UBUNTU_CODENAME" == "jammy" ]]; then
  echo "Ubuntu 22.04 detected. Set ROS_DISTRO to humble."
  ROS_DISTRO="humble"
  ORIGINAL_IMAGE="[1ebe853ca69ce507a69f97bb70f13bc1ffcfa7a2]ubuntu-22.04.2-preinstalled-server-arm64+raspi.img.xz"
  IMAGE_DOWNLOAD_SITE=
else
  echo "Ubuntu $UBUNTU_CODENAME is not supported"
  exit 1
fi

# this is a workaround for the needrestart issue
# ref: https://gist.github.com/fernandoaleman/c3191ed46c977f0a3fcfbdac319183fc
disable_needrestart

echo "The original image: $ORIGINAL_IMAGE"
echo "The original image download site: $IMAGE_DOWNLOAD_SITE"

WORKSPACE="zbotlino_ws"
echo "UBUNTU_CODENAME=$UBUNTU_CODENAME"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "WORKSPACE=$WORKSPACE"
../../ros2/scripts/prepare_ros2_workspace.sh -u "$UBUNTU_CODENAME" -r "$ROS_DISTRO" -w "$WORKSPACE"

source /opt/ros/"$ROS_DISTRO"/setup.bash
ROS_DISTRO="$(printenv ROS_DISTRO)"

# choose from zbotlino, zbotlinoinvert, zbotlinosick1, zbotlinosick2, zbotlino2
# BASE=zbotlinosick2
BASE=zbotlino2

if [ $BASE == "zbotlino" ]; then
  LASER_SENSOR=rplidar
elif [ $BASE == "zbotlinoinvert" ]; then
  LASER_SENSOR=nanoscan3
elif [ $BASE == "zbotlinosick1" ]; then
  LASER_SENSOR=nanoscan3
elif [ $BASE == "zbotlinosick2" ]; then
  LASER_SENSOR=nanoscan3
elif [ $BASE == "zbotlino2" ]; then
  LASER_SENSOR=nanoscan3
else
  echo "Invalid robot type"
  exit 1
fi

# DEPTH_SENSOR=realsense
DEPTH_SENSOR=

function install_rplidar {
  sudo apt install -y ros-"$ROS_DISTRO"-rplidar-ros
  cd /tmp
  wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
  sudo cp rplidar.rules /etc/udev/rules.d/
}

# function install_realsense {
#   sudo apt install -y ros-"$ROS_DISTRO"-realsense2-camera
#   cd /tmp
#   wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
#   sudo cp 99-realsense-libusb.rules /etc/udev/rules.d
# }

function install_realsense_new {
  # sudo apt install -y ros-"$ROS_DISTRO"-realsense2-camera

  cd
  wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
  chmod +x ./libuvc_installation.sh
  ./libuvc_installation.sh

  cd /tmp
  wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
  sudo cp 99-realsense-libusb.rules /etc/udev/rules.d
  sudo udevadm control --reload-rules && sudo udevadm trigger
}

if [[ "$ROS_DISTRO" == "" || "$ROS_DISTRO" == "<unknown>" ]]; then
  echo "No ROS2 distro detected"
  echo "Try running $ source /opt/ros/<ros_distro>/setup.bash and try again."
  exit 1
fi

echo "You are installing zbot_lino on your robot computer."
echo
echo "===========SUMMARY============"
echo "ROBOT TYPE   : $BASE"
echo "LASER SENSOR : $LASER_SENSOR"
echo "DEPTH SENSOR : $DEPTH_SENSOR"
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

echo
echo ====================================================================
echo Prepare VCS sources
echo ====================================================================
# vcs_repo_path="$script_dir/zbot_lino_$ROS_DISTRO.repos"
vcs_repo_path="$script_dir/zbot_linov2_$ROS_DISTRO.repos"
"$prepare_vcs_sh" $WORKSPACE $vcs_repo_path

echo
echo "===================================================================="
echo "Install LIDAR/Depth Sensor ROS2 drivers"
echo "===================================================================="
install_rplidar
install_realsense_new

echo
echo "===================================================================="
echo "Install apt packages"
echo "===================================================================="
# sudo apt install -y python3-vcstool build-essential ros-"$ROS_DISTRO"-robot-localization
sudo apt install -y python3-vcstool build-essential ros-"$ROS_DISTRO"-robot-localization ros-"$ROS_DISTRO"-rosbridge-server
sudo apt install -y python3-websocket # for fitrobot_lino.status.service

echo
echo "===================================================================="
echo "Install micro_ros_setup"
echo "===================================================================="
WORKSPACEPATH="$HOME/$WORKSPACE"
cd "$WORKSPACEPATH"
vcs import src < "$vcs_repo_path"
touch "$WORKSPACEPATH/src/fitrobot"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/fitrobot_interfaces"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/fitrobotcpp"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/realsense-ros"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/sick_safetyscanners2"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/sick_safetyscanners2_interfaces"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/sick_safetyscanners2_base"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/zbot_lino/linorobot2"/COLCON_IGNORE
touch "$WORKSPACEPATH/src/zbot_lino/linorobot2/linorobot2_gazebo"/COLCON_IGNORE

# cd "$WORKSPACEPATH/src/zbot_lino/linorobot2" && touch COLCON_IGNORE
# cd "$WORKSPACEPATH/src/zbot_lino/linorobot2/linorobot2_gazebo" && touch COLCON_IGNORE
cd "$WORKSPACEPATH"
rosdep install --from-path src --ignore-src -y
colcon build && source "$WORKSPACEPATH"/install/setup.bash

echo
echo "===================================================================="
echo "Setup micro-ROS agent"
echo "===================================================================="
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source "$WORKSPACEPATH"/install/setup.bash

echo
echo "===================================================================="
echo "Build zbot_lino"
echo "===================================================================="
# cd "$WORKSPACEPATH/src/zbot_lino/linorobot2" && rm COLCON_IGNORE

rm "$WORKSPACEPATH/src/fitrobot"/COLCON_IGNORE
rm "$WORKSPACEPATH/src/fitrobot_interfaces"/COLCON_IGNORE
rm "$WORKSPACEPATH/src/fitrobotcpp"/COLCON_IGNORE
rm "$WORKSPACEPATH/src/realsense-ros"/COLCON_IGNORE
rm "$WORKSPACEPATH/src/sick_safetyscanners2"/COLCON_IGNORE
rm "$WORKSPACEPATH/src/sick_safetyscanners2_interfaces"/COLCON_IGNORE
rm "$WORKSPACEPATH/src/sick_safetyscanners2_base"/COLCON_IGNORE
rm "$WORKSPACEPATH/src/zbot_lino/linorobot2"/COLCON_IGNORE

cd "$WORKSPACEPATH" && colcon build --symlink-install
source "$WORKSPACEPATH"/install/setup.bash

echo "===================================================================="
echo "Use newests nav2 mppi_controllers                                   "
echo "===================================================================="
../../ros2/scripts/install_mppi_controllers.sh -r $ROS_DISTRO -w $WORKSPACE

## ENV Variables
echo ======== Env Variables ========
if [[ "$BASE" != "ci" ]]; then
  echo "export LINOROBOT2_BASE=$BASE" >> ~/.bashrc
  echo "export LINOROBOT2_LASER_SENSOR=$LASER_SENSOR" >> ~/.bashrc
  echo "export LINOROBOT2_DEPTH_SENSOR=$DEPTH_SENSOR" >> ~/.bashrc
  echo
  if [[ "$ROS_DISTRO" == "galactic" ]]; then
    # echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    append_bashrc "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  elif [[ "$ROS_DISTRO" == "humble" ]]; then
    append_bashrc "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  fi
  echo
  echo "Do you want to add sourcing of linorobot2_ws on your ~/.bashrc?"
  echo -n "Yes [y] or No [n]: "
  read reply
  if [[ "$reply" == "y" || "$reply" == "Y" ]]; then
    append_bashrc "source ${WORKSPACEPATH}/install/setup.bash"
  else
    echo
    echo "Remember to run $ source ${WORKSPACEPATH}/install/setup.bash every time you open a terminal."
  fi
fi

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

echo
echo "===================================================================="
echo "Setup audio"
echo "===================================================================="
./setup_audio.sh

echo
echo "===================================================================="
echo "setup network including additional wifi driver                      "
echo "===================================================================="

./overclock.sh # for pi4

# better to run this script manually
./install_rtl88x2bu.sh
# ./set_network.sh
