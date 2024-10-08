#!/usr/bin/env bash
echo "setup apriltag"
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
colcon_build_sh="$(realpath "$script_dir"/../scripts/colcon_build.sh)"
source "$script_dir/../scripts/argparse_ros.sh"
parse_args "$@"
WORKSPACE=${parsed_args["workspace"]-simulations}
WORKSPACEPATH="$HOME/$WORKSPACE"
[ "$VERBOSE" == true ] && print_args

VCS_REPOS=${parsed_args["VCS_REPOS"]-"apriltag/apriltag.repos"}
VCS_REPOS=$(realpath "$VCS_REPOS")

source "$WORKSPACEPATH/install/setup.bash"
REPOS_APRIL="$script_dir/apriltag/apriltag.repos"
IGNORE_APRIL="nova_carter_docking,my_autodock"
$colcon_build_sh -w $WORKSPACE -v "$REPOS_APRIL" -cdf
cd "$WORKSPACEPATH/src/apriltag" || exit # install apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
$colcon_build_sh -w $WORKSPACE -v "$REPOS_APRIL" -bf -t $TOKEN -i $IGNORE_APRIL
