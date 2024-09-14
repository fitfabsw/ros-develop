#!/bin/bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

# shellcheck source=../scripts/utils.sh
source "$script_dir/../scripts/utils.sh"
# shellcheck source=../scripts/argparse.sh
source "$script_dir/../scripts/argparse.sh"

declare -A arg_desc=(
	["-w,--WORKSPACE"]="Workspace name (default: colcon_ws)"
	["-v,--VCS_REPOS"]="vcs repos"
	["-t,--TOKEN"]="github token"
	["-s,--SYMLINK_INSTALL"]="symlink install (default: false)"
	["-p,--EXPORT_COMPILE_COMMANDS"]="export compile commands (default: false)"
	["-f,--FORCE"]="Delete existing directories if they don't contain the repository being imported (default: false)"
	["-i,--IGNORES"]="ignored packages which are separated by comma. COLCON_IGNORE will be added to the package"
)

declare -A parsed_args
parse_args "$@"

declare -A default_flags=(
	["--FORCE"]=false
	["--SYMLINK_INSTALL"]=false
	["--EXPORT_COMPILE_COMMANDS"]=false
)
FORCE=$(parse_flag "FORCE")
WORKSPACE=${parsed_args["WORKSPACE"]-ros2_ws}
VCS_REPOS=${parsed_args["VCS_REPOS"]}
VCS_REPOS=$(readlink -f "$VCS_REPOS")
TOKEN=${parsed_args["TOKEN"]}
SYMLINK_INSTALL=$(parse_flag "SYMLINK_INSTALL")
EXPORT_COMPILE_COMMANDS=$(parse_flag "EXPORT_COMPILE_COMMANDS")
IGNORES=${parsed_args["IGNORES"]}
IGNORES=$(echo "$IGNORES" | tr ',' ' ')
WORKSPACEPATH="$HOME/$WORKSPACE"

extract_repos() {
	local file="$1"
	local repos=()
	while IFS= read -r line; do
		if [[ "$line" =~ ^[[:space:]]+[^:]+: ]]; then
			repos+=($(echo "$line" | sed 's/:$//' | xargs))
		fi
	done < <(grep -B1 'type: git' "$file" | awk '/^[[:space:]]+[^:]+:/{print; getline; next} /^--$/ {next}')
	echo "${repos[*]}"
}

REPO_NAMES=($(extract_repos "$VCS_REPOS"))

echo "VCS_REPOS: $VCS_REPOS"
echo "REPO_NAMES=${REPO_NAMES[*]}"
echo "WORKSPACE: $WORKSPACE"
echo "WORKSPACEPATH: $WORKSPACEPATH"
echo "SYMLINK_INSTALL: $SYMLINK_INSTALL"
echo "EXPORT_COMPILE_COMMANDS: $EXPORT_COMPILE_COMMANDS"
echo "FORCE: $FORCE"
echo "IGNORES: $IGNORES"

extract_package() {
	local workspace="$1"
	shift
	local repos=("$@")
	local packages=()
	# Iterate through each repo and find package.xml files
	for repo in "${repos[@]}"; do
		# Use a temp variable to store the output from find and while loop
		while IFS= read -r path; do
			package_name=$(basename "$(dirname "$path")")
			packages+=("$package_name")
			# echo "Found package: $package_name at $path"
		done < <(find "$workspace/src/$repo" -name package.xml 2>/dev/null)
	done
	# Remove duplicates and format output
	echo "${packages[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '
}

extract_package_path() {
	local workspace="$1"
	shift
	local repos=("$@")
	local packages_path=()
	# Iterate through each repo and find package.xml files
	for repo in "${repos[@]}"; do
		# Use a temp variable to store the output from find and while loop
		while IFS= read -r path; do
			path=$(dirname "$path")
			packages_path+=("$path")
		done < <(find "$workspace/src/$repo" -name package.xml 2>/dev/null)
	done
	# Remove duplicates and format output
	echo "${packages_path[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '
}

packages_to_install=$(extract_package "$WORKSPACEPATH" "${REPO_NAMES[@]}")
packages_to_install_path=$(extract_package_path "$WORKSPACEPATH" "${REPO_NAMES[@]}")

# echo "packages_to_install:"
# for package in $packages_to_install; do
# 	echo "  $package"
# done
# echo "packages_to_install_path:"
# for package_path in $packages_to_install_path; do
# 	echo "  $package_path"
# done

packages_to_install_remove_ignores=$packages_to_install
for to_ignore in $IGNORES; do
	packages_to_install_remove_ignores=$(echo "$packages_to_install_remove_ignores" | sed "s/$to_ignore//g")
done
echo "packages_to_install_remove_ignores: "
packages_to_install_remove_ignores=$(echo "$packages_to_install_remove_ignores" | tr -s " " | sed 's/^ *//;s/ *$//')
echo "$packages_to_install_remove_ignores"

packages_to_install_path_remove_ignores=""
ignore_array=($IGNORES)
for path in $packages_to_install_path; do
	package=$(basename "$path")
	if [[ ! " ${ignore_array[@]} " =~ " ${package} " ]]; then
		packages_to_install_path_remove_ignores+="$path "
	fi
done

echo "packages_to_install_path_remove_ignores"
for each in $packages_to_install_path_remove_ignores; do
	echo "  $each"
done

echo "# Prepare VCS sources"
prepare_vcs_sh="$(realpath "$script_dir"/prepare_vcs.sh)"
"$prepare_vcs_sh" "$WORKSPACE" "$VCS_REPOS" || exit_code=$?
if [[ $exit_code -ne 0 ]]; then
	echo "prepare_vcs_sh failed"
	exit
fi

cp "$VCS_REPOS" "$VCS_REPOS.token"
if [ -n "$TOKEN" ]; then
	echo "Replace github with $TOKEN@github"
	sed -i "s/github/$TOKEN@github/g" "$VCS_REPOS.token"
else
	echo "Please provide a github token in order to download the repositories."
	exit
fi

echo "# Build from source"
cd "$WORKSPACEPATH" || exit
if [ "$FORCE" == true ]; then
	echo "vcs import --force src < $VCS_REPOS.token"
	vcs import --force src <"$VCS_REPOS.token"
else
	echo "vcs import src < $VCS_REPOS.token"
	vcs import src <"$VCS_REPOS.token"
fi

cd "$WORKSPACEPATH" || exit
for path in $packages_to_install_path_remove_ignores; do
	echo "rosdep install --from-paths $path --ignore-src -r -y"
	rosdep install --from-paths $path --ignore-src -r -y
done

cd "$WORKSPACEPATH" || exit
if [ "$EXPORT_COMPILE_COMMANDS" == true ]; then
	if [ "$SYMLINK_INSTALL" == true ]; then
		cmd="colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select $packages_to_install --packages-ignore $IGNORES"
	else
		cmd="colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select $packages_to_install --packages-ignore $IGNORES"
	fi
else
	if [ "$SYMLINK_INSTALL" == true ]; then
		cmd="colcon build --symlink-install --packages-select $packages_to_install --packages-ignore $IGNORES"
	else
		cmd="colcon build --packages-select $packages_to_install --packages-ignore $IGNORES"
	fi
fi
echo "build command:"
echo "$cmd"
eval "$cmd"
