#!/bin/bash
source ../scripts/utils.sh

ORIGINAL_IMAGE="ubuntu-20.04.4-desktop-amd64.iso"
IMAGE_DOWNLOAD_SITE=
ROS_DISTRO="galactic"
WORKSPACE="colcon_ws"

echo "The original image: $ORIGINAL_IMAGE"
echo "The original image donwload site: $IMAGE_DOWNLOAD_SITE"

echo
echo ====================================================================
echo Install ROS2
echo ====================================================================
./install_ros2.sh --ROS_DISTRO=$ROS_DISTRO

echo
echo ====================================================================
echo Install turtlebot4
echo ====================================================================
./install_tb4.sh

echo
echo ====================================================================
echo Install additional ROS packages
echo ====================================================================
ros_packages=(
  "ros-$ROS_DISTRO-turtlesim"
)
install_ubuntu_packages "${ros_packages[@]}"

echo
echo ====================================================================
echo Build additional ROS packages from source
echo ====================================================================
source1="https://github.com/ros/ros_tutorials"
package1_location="~/$WORKSPACE/src/ros_tutorials"
git clone "$source1" -b "$ROS_DISTRO-devel" "$package1_location"
cd "~/$WORKSPACE"
# find $package1_location -type f -name "COLCON_IGNORE" -exec rm {} \;
colcon build --symlink-install

echo
echo ====================================================================
echo Append bashrc file
echo ====================================================================
append_bashrc 'source /opt/ros/galactic/setup.bash'

append_bashrc 'export LIBGL_ALWAYS_SOFTWARE=1'
append_bashrc 'export OGRE_RTT_MODE=Copy'

append_bashrc 'ROS_DOMAIN_ID=0'
append_bashrc 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
append_bashrc 'CYCLONEDDS_URI=/etc/turtlebot4/cyclonedds_pc.xml'
