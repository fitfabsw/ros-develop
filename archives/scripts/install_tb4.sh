#!/bin/bash
#

echo ===========================
echo Install turtlebot4
# references: https://idminer.com.tw/knowledge-base/tb4-overview/
echo ===========================
sudo apt install ros-galactic-turtlebot4-desktop
sudo apt install ros-galactic-turtlebot4-base \
  ros-galactic-turtlebot4-bringup \
  ros-galactic-turtlebot4-description \
  ros-galactic-turtlebot4-diagnostics \
  ros-galactic-turtlebot4-msgs \
  ros-galactic-turtlebot4-navigation \
  ros-galactic-turtlebot4-node \
  ros-galactic-turtlebot4-robot \
  ros-galactic-turtlebot4-tests

echo ===========================
echo "Install gazebo (ignition)"
# references: https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html#ignition-bringup
echo ===========================
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-edifice

# meta packages
sudo apt install ros-galactic-turtlebot4-simulator ros-galactic-irobot-create-nodes
