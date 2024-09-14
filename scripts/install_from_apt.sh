#!/bin/bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
install_ros_package_sh="$(readlink -f $script_dir/install_ros_packages.sh)"
source "$script_dir/../scripts/utils.sh"

WORKSPACE="${1:-colcon_ws}"
ROS_DISTRO="${2:-galactic}"
APT_PACKAGE_FILE=$3

echo
echo ====================================================================
echo Basic check
echo ====================================================================
ros_package_file="$APT_PACKAGE_FILE"

if [ -d $HOME/"$WORKSPACE" ]; then
  if [ -f "$ros_package_file" ]; then
    echo "ros_package_file: $ros_package_file exists"
  else
    echo "ERROR: ros_package_file $ros_package_file does not exist"
    exit 1
  fi
  source "$ros_package_file"
else
  echo "ERROR: WORKSPACE $WORKSPACE does not exist"
  exit 1
fi

echo
echo ====================================================================
echo Install ROS packages for ROS_DISTRO "$ROS_DISTRO"
echo ====================================================================
"$install_ros_package_sh" "$ROS_DISTRO" "${ros_packages[@]}"

# copy cyclonedds config file
# cp "cyclonedds.xml" $HOME/cyclonedds.xml
# append_bashrc 'export CYCLONEDDS_URI=$HOME/cyclonedds.xml'
