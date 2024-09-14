#!/bin/bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

# shellcheck source=../scripts/utils.sh
source "$script_dir/../scripts/utils.sh"
# shellcheck source=../scripts/argparse.sh
source "$script_dir/../scripts/argparse.sh"

declare -A arg_desc=(
  ["-w,--WORKSPACE"]="Workspace name (default: colcon_ws)"
  ["-v,--VCS_REPOS"]="vcs repos"
  ["-f,--force"]="Delete existing directories if they don't contain the repository being imported (default: false)"
  ["-i,--IGNORES"]="ignored packages which are separated by comma. COLCON_IGNORE will be added to the package"
)

declare -A parsed_args
parse_args "$@"

declare -A default_flags=(
  ["--force"]=false
)
FORCE=$(parse_flag "force")
WORKSPACE=${parsed_args["WORKSPACE"]-ros2_ws}
VCS_REPOS=${parsed_args["VCS_REPOS"]}
VCS_REPOS=$(readlink -f "$VCS_REPOS")
IGNORES=${parsed_args["IGNORES"]}
IFS=',' read -ra ignored_ros_packages <<<"$IGNORES"

echo ====== install_from_source ======
echo "WORKSPACE: $WORKSPACE"
echo "VCS_REPOS: $VCS_REPOS"
echo "FORCE: $FORCE"
if [ -n "$IGNORES" ]; then
  echo "IGNORES: "
  for item in "${ignored_ros_packages[@]}"; do
    echo "$item"
  done
fi

echo
echo ====================================================================
echo Prepare VCS sources
echo ====================================================================
prepare_vcs_sh="$(realpath "$script_dir"/prepare_vcs.sh)"
"$prepare_vcs_sh" "$WORKSPACE" "$VCS_REPOS" || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
  echo "prepare_vcs_sh failed"
  exit
fi

echo
echo ====================================================================
echo Build from source
echo ====================================================================
WORKSPACEPATH="$HOME/$WORKSPACE"
cd "$WORKSPACEPATH" || exit
if [ "$FORCE" == true ]; then
  echo "vcs import --force src < $VCS_REPOS"
  vcs import --force src <"$VCS_REPOS"
else
  echo "vcs import src < $VCS_REPOS"
  vcs import src <"$VCS_REPOS"
fi

for pkg in "${ignored_ros_packages[@]}"; do
  # pkg_path=$(find src -type d -name $pkg)
  pkg_path=$(find src -type d -name "$pkg" | head -n 1)
  echo "$pkg --> $pkg_path"
  cd "$WORKSPACEPATH/$pkg_path" && touch COLCON_IGNORE
  cd "$WORKSPACEPATH" || exit
done

rosdep install --from-paths src --ignore-src -r -y

# colcon build --symlink-install

# This is only for avoiding dwb_critics build error, temp
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"

echo ====================================================================
echo Append bashrc file
echo ====================================================================
append_bashrc "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
