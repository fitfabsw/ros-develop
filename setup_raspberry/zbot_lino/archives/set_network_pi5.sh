#!/usr/bin/env bash
script_dir="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$script_dir/../../scripts/utils.sh"

# get env BASE
BASE=${LINOROBOT2_BASE-"zbotlino2"}

# copy netplan config file
filepath="/etc/netplan/50-cloud-init.yaml"
sudo cp "$filepath" "$filepath.bak"
sudo cp "50-cloud-init-$BASE-pi5.yaml" "$filepath"

# copy cyclonedds config file
cp "cyclonedds-$BASE-pi5.xml" $HOME/cyclonedds.xml
append_bashrc 'export CYCLONEDDS_URI=$HOME/cyclonedds.xml'
