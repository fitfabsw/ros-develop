#!/usr/bin/bash
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../scripts/utils.sh"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../scripts/argparse_ros.sh"
parse_args "$@"

LATEST_WORKED_COMMIT="1.1.12"

echo "ROSDISTRO=$ROSDISTRO"
echo "WORKSPACE=$WORKSPACE"
echo "LATEST_WORKED_COMMIT=$LATEST_WORKED_COMMIT"

if [[ $ROSDISTRO == "humble" ]]; then
  echo "Install mppi_controllers from source."
  # when comping from source, trying out several commit, below is the latest results
  #
  # humble (57f55c) failed (10/16)
  # 3b791e failed (10/13)
  # 1b97e3 failed [MPPI Optimization] (10/10)
  # 1.1.12 ok (10/4)
  # 1.1.11 failed
  # 1.1.10 ok
  # 1.1.9 ok (8/4)

  # ref: MPPI crashing on loading plug-ings #3767
  # https://github.com/ros-planning/navigation2/issues/3767
  #
  rm -rf /tmp/navigation2 > /dev/null 2>&1
  git clone https://github.com/ros-planning/navigation2 /tmp/navigation2 -b "$ROSDISTRO"
  cd /tmp/navigation2 && git checkout $LATEST_WORKED_COMMIT
  cp -R /tmp/navigation2/nav2_mppi_controller "$HOME/$WORKSPACE/src"
fi
