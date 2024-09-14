#!/bin/bash
echo "setup turtlebot3 simulation"
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
install_from_source_sh="$(realpath "$script_dir"/../../scripts/install_from_source.sh)"
source "$script_dir/../../scripts/argparse_ros.sh"
parse_args "$@"
WORKSPACE=${parsed_args["workspace"]-simulations}
[ "$VERBOSE" == true ] && print_args

source /opt/ros/${ROSDISTRO}/setup.bash >/dev/null 2>&1 || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
  echo "/opt/ros/$ROSDISTRO/setup.sh does not exist."
  print_usage
  exit
fi

echo
echo ====================================================================
echo Install turtlebot3 from source
echo ====================================================================
# "$install_from_source_sh" -w "$WORKSPACE" -v "$VCS_REPOS"
"$install_from_source_sh" -w "$WORKSPACE" -v "$script_dir/tb3-$ROSDISTRO.repos"
