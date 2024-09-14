#!/usr/bin/env bash
echo "setup zbot lino simulation"
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
install_from_source_sh="$(realpath $script_dir/../../scripts/install_from_source.sh)"
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

[ $MPPI == true ] && "$(realpath $script_dir/../../ros2/scripts/install_mppi_controllers.sh)" -w $WORKSPACE
[ $RMF == true ] && "$(realpath $script_dir/../install_rmf.sh)" -w $WORKSPACE

cp "$script_dir/zbot_lino_$ROSDISTRO.repos" "$script_dir/zbot_lino_$ROSDISTRO.repos.token"

if [ -n "$TOKEN" ]; then
  echo "Replace github with $TOKEN@github"
  sed -i "s/github/$TOKEN@github/g" "$script_dir/zbot_lino_$ROSDISTRO.repos.token"
else
  echo "Please provide a github token in order to download the repositories."
  exit
fi

"$install_from_source_sh" -w $WORKSPACE -v "$script_dir/zbot_lino_$ROSDISTRO.repos.token" $([ $FORCE == true ] && echo "-f") -i linorobot2_bringup
