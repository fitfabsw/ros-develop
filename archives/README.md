## Setup zbot on Raspberry Pi 4B

Please refer to [Install zbot_artic & zbot_lino](setup_raspberry/README.md) for more details.

## Quickly build envirnoment for ROS1/ROS2 developement.

Please git clone this repo into you developement envirnoment.

## Install ROS2

### For the virtual machine envirnoment

The host machine is MacOS, and the virtual machine is Ubuntu 20.04 installed through Parallels.

### Ubuntu source image

The source image is from (Source: http://old-releases.ubuntu.com/releases/)

#### Ubuntu 20.04

The image used is ubuntu-20.04.5-desktop-amd64.iso

(http://old-releases.ubuntu.com/releases/20.04.5/ubuntu-20.04.5-desktop-amd64.iso)

```bash
# in your virtual machine
$ git clone https://github.com/zealzel/ros-develop.git $HOME
$ cd $HOME/ros-develop/ros2/scripts

# install ROS2
$ ./install_ros2.sh --ROS_DISTRO=galactic --ROS_INSTALL_TYPE=desktop --APPEND_SOURCE_SCRIPT_TO_BASHRC=true
or
$ ./install_ros2.sh -r galactic -i desktop -a

# install colcon
$ ./install_colcon.sh --ROS_DISTRO=galactic
```

### Setup simulation envirnoment

```bash
$ cd $HOME/ros-develop/setup_vm
$ ./setup_simulations.sh
```

Currently, the following simulation envirnoment are supported:

**simulations_articubot_one**

![localImage](./articubot_obstacles.gif)

**simulations_tiago**

![localImage](./tiago_house.gif)

Please follow the snippets in each simulations_xxx folder to test the functions.
