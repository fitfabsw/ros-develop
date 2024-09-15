#!/usr/bin/env bash
echo "setup zbot artic simulation"
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
colcon_build_sh="$(realpath "$script_dir"/../../bin/colcon_build)"
source "$script_dir/../../scripts/argparse_ros.sh"
parse_args "$@"
WORKSPACE=${parsed_args["workspace"]-simulations}
WORKSPACEPATH="$HOME/$WORKSPACE"
[ "$VERBOSE" == true ] && print_args

source /opt/ros/${ROSDISTRO}/setup.bash >/dev/null 2>&1 || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
	echo "/opt/ros/$ROSDISTRO/setup.sh does not exist."
	print_usage
	exit
fi

echo -e "\n=========colcon build artic=============="
source "$WORKSPACEPATH/install/setup.bash"
REPOS_ARTIC="$script_dir/zbot_artic_$ROSDISTRO.repos"
IGNORE_ARTIC="gazebo_ros2_control_demos"
cmd="$colcon_build_sh -w $WORKSPACE -r "$REPOS_ARTIC" -cdbf --CLEAN_CACHE -i $IGNORE_ARTIC ${TOKEN:+-t$TOKEN}"
echo "$cmd"
eval "$cmd"
