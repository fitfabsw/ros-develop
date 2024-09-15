#!/usr/bin/env bash
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
install_from_source_sh="$(realpath $script_dir/../scripts/install_from_source.sh)"
install_from_apt_sh="$(realpath "$script_dir"/../scripts/install_from_apt.sh)"
source "$script_dir/../scripts/argparse_ros.sh"
parse_args "$@"
WORKSPACE=${parsed_args["workspace"]-simulations}
[ "$VERBOSE" == true ] && print_args

if [[ "$UBUNTU_CODENAME" == "focal" ]]; then
  echo "Ubuntu 20.04 detected. Set ROSDISTRO to galactic."
elif [[ "$UBUNTU_CODENAME" == "jammy" ]]; then
  echo "Ubuntu 22.04 detected. Set ROSDISTRO to humble."
elif [[ "$UBUNTU_CODENAME" == "noble" ]]; then
  echo "Ubuntu 24.04 detected. Set ROSDISTRO to jazzy."
fi

echo ===============================================
echo Prepare workspace
echo ===============================================
"$(realpath "$script_dir"/../scripts/create_workspace.sh)" "$WORKSPACE" || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
  exit
fi

if [ "$ROS2_DEV" == true ]; then
  echo ===============================================
  echo Prepare ROS2 development environment
  echo ===============================================
  "$(realpath "$script_dir"/../ros2/scripts/prepare_ros2_workspace.sh)" -w "$WORKSPACE"
  source /opt/ros/${ROSDISTRO}/setup.bash >/dev/null 2>&1 || exit_code=$?
  if [[ $exit_code -ne 0 ]]; then
    echo "/opt/ros/$ROSDISTRO/setup.sh does not exist."
    print_usage
    exit
  fi
fi

[ "$MPPI" == true ] && "$(realpath "$script_dir"/../ros2/scripts/install_mppi_controllers.sh)" -w "$WORKSPACE"

if [ "$DOWNLOAD_GZ" == true ]; then
  echo ===============================================
  echo Download gazebo classic models
  echo ===============================================
  "$(realpath "$script_dir"/download_gazebo_models.sh)"
fi

echo ===============================================
echo 1. Install robots from package manager
echo ===============================================
"$install_from_apt_sh" "$WORKSPACE" "$ROSDISTRO" "$script_dir"/simulations_turtlebot3/ros_packages.sh

echo ===============================================
echo 2. Build/Install robots/worlds from source
echo ===============================================
"$script_dir"/simulations_zbot_lino/setup_zbot_lino.sh -w "$WORKSPACE" "${TOKEN:+-t$TOKEN}" "$([ "$FORCE" == true ] && echo "-f")"
"$script_dir"/simulations_zbot_artic/setup_zbot_artic.sh -w "$WORKSPACE" "${TOKEN:+-t$TOKEN}" "$([ "$FORCE" == true ] && echo "-f")"
# "$install_from_source_sh" $WORKSPACE "$script_dir/simulations_neobotix/neobotix.repos"
# "$install_from_source_sh" $WORKSPACE "$script_dir/world_aws_robotmaker/deps.repos"

echo ===============================================
echo 3. Build/Install robots by customed scripts
echo ===============================================
[ "$RMF" == true ] && "$(realpath "$script_dir"/install_rmf.sh)" -w "$WORKSPACE" "$([ "$FORCE" == true ] && echo "-f")"
[ "$DOCK" == true ] && "$(realpath "$script_dir"/install_apriltag.sh)" -w "$WORKSPACE"

WORKSPACEPATH="$HOME/$WORKSPACE"
rm -f "$WORKSPACEPATH"/*.repos
rm -f "$WORKSPACEPATH"/*.rosinstall

echo ===============================================
echo Set env variables
echo ===============================================
# [ $APPENDBASHRC == true ] && "$(realpath $script_dir/ros2_append_bashrc.sh)"
"$(realpath "$script_dir"/ros2_append_bashrc.sh)"
