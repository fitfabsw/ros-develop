title() {
  description=$1
  echo
  echo ====================================================================
  echo "$description"
  echo ====================================================================
}

check_last_command() {
  if [ $? -ne 0 ]; then
    return 1 # 返回非0值表示失败
  fi
  return 0
}

check_exit_code() {
  exit_code=$1
  description=$2
  if [[ $exit_code -ne 0 ]]; then
    echo "$description failed with exit code $exit_code"
    print_elapsed_summary
    exit
  fi
}

# 计算并存储时间
calculate_and_store_time() {
  local start_time=$1
  local stage_name=$2
  local end_time=$(date +%s)
  local elapsed=$((end_time - start_time))

  # 将阶段名和耗时分别添加到各自的数组
  stage_names+=("$stage_name")
  elapsed_times+=("$elapsed")
}

# 打印耗时摘要
print_elapsed_summary() {
  echo -e "\n=== Elapsed time summary ==="
  local len=${#stage_names[@]}
  for ((i = 0; i < $len; i++)); do
    echo "${stage_names[$i]}: ${elapsed_times[$i]} seconds"
  done
}

# calculate_and_store_time() {
#     local start_time=$1
#     local stage_name=$2
#     local end_time=$(date +%s)
#     local elapsed=$((end_time - start_time))
#     elapsed_times["$stage_name"]=$elapsed
# }

# print_elapsed_summary() {
#     echo "Elapsed time summary:"
#     for stage in "${!elapsed_times[@]}"; do
#         echo "$stage: ${elapsed_times[$stage]} seconds"
#     done
# }

idpt-append-sudo() {
  line="$1"
  file="$2"
  if ! grep -Fxq "$line" "$file"; then
    echo "$line" | sudo tee -a "$file"
  fi
}

idpt-append() {
  line="$1"
  file="$2"
  if ! grep -Fxq "$line" "$file"; then
    echo "$line" >>"$file"
  fi
}

append_bashrc() {
  file="$HOME/.bashrc"
  lines=("$@")
  echo "file: $file"
  for line in "${lines[@]}"; do
    echo "line: \"$line\" added"
    idpt-append "$line" "$file"
  done
}

append_content_lines() { # works for multi-line content
  file="$1"
  multi_line_content="$2"
  echo "file: $file"
  # Here, we use 'echo' and 'while' to read each line from the string.
  echo "$multi_line_content" | while IFS= read -r line; do
    echo "Appending line: \"$line\""
    idpt-append "$line" "$file" # appending content line-by-line
  done
}

# Now, we modify the 'append_content' to append instead of overwrite
append_content() {
  file="$1"
  new_content="$2"
  echo "file: $file"
  # Directly append new content to the file without overwriting
  append_content_lines "$file" "$new_content"
}

# write a function which tell if a ubuntu package is installed
is_ubuntu_package_installed() {
  if dpkg -s "$1" >/dev/null 2>&1; then
    echo "Package $1 is installed"
    return 0
  else
    echo "Package $1 is not installed"
    return 1
  fi
}

is_ubuntu_packages_installed() {
  packages=("$@")
  for pkg in "${packages[@]}"; do
    if ! is_ubuntu_package_installed "$pkg"; then
      return 1
    fi
  done
}

upgrade_ubuntu_packages() {
  ensure_sudo
  packages=("$@")
  for pkg in "${packages[@]}"; do
    if ! is_ubuntu_package_installed "$pkg"; then
      sudo apt-get upgrade -y "$pkg"
    fi
  done
}

install_ubuntu_packages() {
  ensure_sudo
  packages=("$@")
  for pkg in "${packages[@]}"; do
    if ! is_ubuntu_package_installed "$pkg"; then
      sudo apt-get install -y "$pkg"
    fi
  done
}

ensure_sudo() {
  if [[ $(dpkg -L sudo) ]]; then
    echo "sudo is installed"
  else
    echo "sudo is not installed, try install it"
    apt-get update
    apt-get install -y sudo || exit_code=$?
    if [[ $exit_code -ne 0 ]]; then
      echo "install sudo failed."
    else
      echo "install sudo done."
    fi
  fi
}

disable_needrestart() {
  ensure_sudo
  if [[ $(dpkg -l | grep needrestart) ]]; then
    echo "needrestart is installed"
    sudo sed -i 's/#$nrconf{restart} = '"'"'i'"'"';/$nrconf{restart} = '"'"'a'"'"';/g' /etc/needrestart/needrestart.conf

    # ref: https://askubuntu.com/questions/1349884/how-to-disable-pending-kernel-upgrade-message
    sudo sed -i "s/#\$nrconf{kernelhints} = -1;/\$nrconf{kernelhints} = -1;/g" /etc/needrestart/needrestart.conf
  fi
}

ros2_colcon_ignore() {
  WORKSPACE=$1
  touch $HOME/$WORKSPACE/src/ament/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/eclipse-cyclonedds/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/eclipse-iceoryx/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/eProsima/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/ignition/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/joint_state_publisher/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/osrf/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/ros/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/ros2/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/ros-perception/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/ros-planning/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/ros-tooling/COLCON_IGNORE
  touch $HOME/$WORKSPACE/src/ros-visualization/COLCON_IGNORE
}

ros2_colcon_rm_ignore() {
  WORKSPACE=$1
  rm $HOME/$WORKSPACE/src/ament/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/eclipse-cyclonedds/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/eclipse-iceoryx/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/eProsima/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/ignition/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/joint_state_publisher/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/osrf/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/ros/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/ros2/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/ros-perception/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/ros-planning/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/ros-tooling/COLCON_IGNORE >/dev/null 2>&1
  rm $HOME/$WORKSPACE/src/ros-visualization/COLCON_IGNORE >/dev/null 2>&1
}
