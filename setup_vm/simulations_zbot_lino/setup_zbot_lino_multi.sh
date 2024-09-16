#!/usr/bin/env bash
echo "setup zbot lino simulation"
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

[ $MPPI == true ] && "$(realpath $script_dir/../../ros2/scripts/install_mppi_controllers.sh)" -w $WORKSPACE
[ $RMF == true ] && "$(realpath $script_dir/../install_rmf.sh)" -w $WORKSPACE

echo -e "\n=========colcon build lino2=============="
source "$WORKSPACEPATH/install/setup.bash"
REPOS_LINO2="$script_dir/zbot_lino_"$ROSDISTRO"_multi.repos"
IGNORE_LINO2="linorobot2_bringup,linorobot2_gazebo,fitrobot_interfaces"
#
cmd="$colcon_build_sh -w $WORKSPACE -r "$REPOS_LINO2" -cdf ${TOKEN:+-t$TOKEN}"
echo "# clone repos"
echo "$cmd"
eval "$cmd"
#
cd $WORKSPACEPATH
echo "# build repos that only ues colcon build without symlink-install"
cmd="colcon build --packages-select linorobot2_gazebo fitrobot_interfaces --cmake-clean-cache"
echo "$cmd"
eval "$cmd"
#
echo "# build the rest repos that ues colcon build --symlink-install"
cmd="$colcon_build_sh -w $WORKSPACE -r "$REPOS_LINO2" -bs --CLEAN_CACHE -i $IGNORE_LINO2"
echo "$cmd"
eval "$cmd"
