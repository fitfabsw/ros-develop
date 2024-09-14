#!/bin/bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$script_dir/../../scripts/argparse_ros.sh"
source "$script_dir/../../scripts/utils.sh"

echo
echo ====================================================================
echo Install ROS2
echo ====================================================================
"$script_dir/install_ros2.sh"
"$script_dir/install_ros2_packages.sh"

ensure_rosdep_init
# echo
# echo ====================================================================
# echo Ensure rosdep is initialized
# echo ====================================================================
# rosdep update || exit_code=$?
# if [[ $exit_code -ne 0 ]]; then
# 	sudo rosdep init
# 	rosdep update --include-eol-distros
# fi
