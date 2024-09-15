#!/usr/bin/env bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

# shellcheck source=../scripts/utils.sh
source "$script_dir/../scripts/utils.sh"
# shellcheck source=../scripts/argparse.sh
source "$script_dir/../scripts/argparse.sh"
source "$script_dir/../scripts/utils.sh"

declare -A arg_desc=(
	["-w,--WORKSPACE"]="Workspace name (default: colcon_ws)"
	["-v,--VCS_REPOS"]="vcs repos (default: apriltag/apriltag.repos)"
)

declare -A parsed_args
parse_args "$@"

WORKSPACE=${parsed_args["WORKSPACE"]-ros2_ws}
VCS_REPOS=${parsed_args["VCS_REPOS"]-"apriltag/apriltag.repos"}
VCS_REPOS=$(realpath "$VCS_REPOS")

echo ====== install_from_source ======
echo "WORKSPACE: $WORKSPACE"
echo "VCS_REPOS: $VCS_REPOS"

echo ===============================================
echo Prepare workspace
echo ===============================================
"$(realpath "$script_dir"/../scripts/create_workspace.sh)" "$WORKSPACE" || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
	exit
fi

echo ===============================================
echo Clone repositories through vcstool
echo ===============================================
WORKSPACEPATH="$HOME/$WORKSPACE"
cd "$WORKSPACEPATH" || exit
# alway using --force flag
echo "vcs import --force src < $VCS_REPOS"
vcs import --force src <"$VCS_REPOS"

# install dependencies
# apriltag
# apriltag_ros
# my_autodock
# opennav_docking
cd "$WORKSPACEPATH" || exit
ensure_rosdep_init
# rosdep install --from-paths src --ignore-src -r -y
# rosdep install --from-paths src/{apriltag,apriltag_ros,my_autodock,opennav_docking} --ignore-src -r -y
rosdep install --from-paths src/{apriltag,apriltag_ros,opennav_docking} --ignore-src -r -y

# ignore compiling no-used in opennav_docking
touch "$WORKSPACEPATH/src/opennav_docking/nova_carter_docking"/COLCON_IGNORE

# install apriltag
cd "$WORKSPACEPATH/src/apriltag" || exit
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install

# build the rest repos
cd "$WORKSPACEPATH" || exit
# colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
"$(realpath "$script_dir"/../scripts/colcon_build.sh)" -w $WORKSPACE -v "$VCS_REPOS" -t $TOKEN -f
