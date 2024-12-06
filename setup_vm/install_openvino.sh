#!/bin/bash

# references
# https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-apt.html

# Install the GPG key for the repository
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

# Add this key to the system keyring:
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

# Add the repository via the following command:
# ubuntu 24.04
# echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu24 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list
# ubuntu 22.04
echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list
# ubuntu 20.04
# echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu20 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list

sudo apt update
apt-cache search openvino

# Install OpenVINO Runtime
sudo apt install openvino

# Check for Installed Packages and Versions
apt list --installed | grep openvino
