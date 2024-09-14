#!/usr/bin/env bash
script_dir="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
install_from_source_sh="$(realpath $script_dir/../scripts/install_from_source.sh)"
source "$script_dir/../scripts/utils.sh"
source "$(realpath "$script_dir/../scripts/argparse_ros.sh")"

ros_packages=(
  # rmf
  "ros-$ROSDISTRO-rmf-dev"
  # traffic-editor
  "ros-$ROSDISTRO-rmf-traffic-editor"
  "ros-$ROSDISTRO-rmf-building-map-tools"
  "ros-$ROSDISTRO-rmf-traffic-editor-assets"
  "ros-$ROSDISTRO-rmf-traffic-editor-test-maps"
)

echo ===============================================
echo Install from package manager
echo ===============================================
install_ubuntu_packages "${ros_packages[@]}"

echo ===============================================
echo Build/Install from source
echo ===============================================
if [[ "$ROSDISTRO" == "humble" ]]; then
  # temporary ignore ignition related packages since they will fail during building
  "$install_from_source_sh" -w "$WORKSPACE" -v "$script_dir/rmf/rmf_$ROSDISTRO.repos" $([ $FORCE == true ] && echo "-f") -i rmf_building_sim_gz_plugins,rmf_robot_sim_gz_plugins
fi
