#!/usr/bin/env bash
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
colcon_build_sh="$(realpath "$script_dir"/../bin/colcon_build)"
source "$script_dir/../scripts/argparse_ros.sh"
parse_args "$@"
WORKSPACE=${parsed_args["workspace"]-simulations}
WORKSPACEPATH="$HOME/$WORKSPACE"
[ "$VERBOSE" == true ] && print_args

echo ===============================================
echo Prepare workspace
echo ===============================================
"$(realpath "$script_dir"/../scripts/create_workspace.sh)" "$WORKSPACE" || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
	exit
fi

# echo -e "\n=========install apriltag=============="
# source "$WORKSPACEPATH/install/setup.bash"
# REPOS_APRIL="$script_dir/apriltag/apriltag.repos"
# IGNORE_APRIL="nova_carter_docking,my_autodock"
# $colcon_build_sh -w $WORKSPACE -r "$REPOS_APRIL" -cdf ${TOKEN:+-t$TOKEN}
# cd "$WORKSPACEPATH/src/apriltag" || exit # install apriltag
# cmake -B build -DCMAKE_BUILD_TYPE=Release
# sudo cmake --build build --target install
# $colcon_build_sh -w $WORKSPACE -r "$REPOS_APRIL" -b --CLEAN_CACHE -i $IGNORE_APRIL ${TOKEN:+-t$TOKEN}

# echo -e "\n=========colcon build lino2=============="
# source "$WORKSPACEPATH/install/setup.bash"
# REPOS_LINO2="$script_dir/simulations_zbot_lino/zbot_lino_"$ROSDISTRO"_multi.repos"
# IGNORE_LINO2="linorobot2_bringup"
# $colcon_build_sh -w $WORKSPACE -r "$REPOS_LINO2" -cdbf --CLEAN_CACHE -i $IGNORE_LINO2 ${TOKEN:+-t$TOKEN}

# echo -e "\n=========colcon build artic=============="
# source "$WORKSPACEPATH/install/setup.bash"
# REPOS_ARTIC="$script_dir/simulations_zbot_artic/zbot_artic_"$ROSDISTRO"_multi.repos"
# IGNORE_ARTIC="gazebo_ros2_control_demos"
# $colcon_build_sh -w $WORKSPACE -r "$REPOS_ARTIC" -cdbf --CLEAN_CACHE -i $IGNORE_ARTIC ${TOKEN:+-t$TOKEN}
