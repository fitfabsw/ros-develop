#### Setup script

under setup_vm, there is a script setup_simulations.sh, which is used to setup the simulation environment.

```bash
./setup_simulations.sh -h

Usage: ./setup_simulations.sh [OPTIONS]

Options:
  -a,--append_bashrc         append bashrc (default: false)
  -f,--force                 delete workspace repositories (default: false)
  -g,--download_gz_models    download gazebo models (default: false)
  -h,--help                  help
  -i,--ros_install_type      the type of ROS installation (desktop|ros-base) (default: desktop)
  -m,--enable_mppi_fix       enable mppi fix (default: false)
  -o,--install_ros2          install ros2 dev environment (default: false)
  -r,--enable_rmf            enable rmf environment (default: false)
  -d,--enable_autodock       enable auto-docking environment (default: false)
  -t,--token                 github token
  -v,--verbose               verbose (default: false)
  -w,--workspace             Workspace name (default: my_ws)
```

#### From scratch

```bash
./setup_simulations.sh -t <token> --install_ros2
```

It will do the following:

- Prepare workspace

  by default, it will create a workspace named simulations

- Prepare ROS2 development environment

- Install robots from package manager

  It will install turtlebot3 & related gazebo environments

- Build/Install robots/worlds from source

  It will install robots artic & lino2

##### Addtionally, you can

- include MPPI_controller fix

  `./setup_simulations.sh -t <token> --enable_mppi_fix`

- Download gazebo classic models

  `./setup_simulations.sh -t <token> --download_gz_models`

- Install OpenRMF related environments

  `./setup_simulations.sh -t <token> --enable_rmf`

- Set env variables

  `./setup_simulations.sh -t <token> --append_bashrc

off course, you can combine them as you like

`./setup_simulations.sh -t <token> -mdra`

#### Install zbot separately

##### Install zbot artic

cd into folder setup_vm/simulations_zbot_artic

```bash
./setup_zbot_artic.sh -t <token>
```

##### Install zbot lino2

cd into folder setup_vm/simulations_zbot_lino

```bash
./setup_zbot_lino.sh -t <token>
```
