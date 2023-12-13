# Autoware Docker Installation Guide

This documentation covers `Docker Installation of Autoware` and testing the installation with `Autoware Planning Simulation`.

> **Note**: Installation was tested with Host-PC running Ubuntu 20.04 (with ROS 1 Noetic, ROS 2 Foxy & ROS 2 Galactic) and Docker-Container running Ubuntu 22.04 (with ROS 2 Humble).

## Official Documentation:

For Autoware's general documentation, see [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/).

For detailed documents of Autoware Universe components, see [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/).

For official Autoware installation guide, see [Autoware Installation Guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/).
  - [Local (Source) Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
  - [Docker Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)

For official Docker installation guide, see [Get Docker](https://docs.docker.com/get-docker/), and particularly for Ubuntu, see [Docker Ubuntu Installation](https://docs.docker.com/desktop/install/ubuntu/).

---

## Host PC:

```bash
user@host-pc:~$ mkdir -p Autoware_WS

user@host-pc:~$ cd Autoware_WS

user@host-pc:~$ git clone https://github.com/autowarefoundation/autoware.git

user@host-pc:~$ sudo mv $HOME/Autoware_WS/autoware $HOME/Autoware_WS/autoware_docker

user@host-pc:~$ cd autoware_docker

user@host-pc:~$ ./setup-dev-env.sh docker

user@host-pc:~$ # Setting up the build environment can take up to 1 hour.
                        # >  Are you sure you want to run setup? [y/N]
                        y

user@host-pc:~$ # [Warning] Some Autoware components depend on the CUDA, cuDNN and TensorRT NVIDIA libraries which have end-user license agreements that should be reviewed before installation.
                        # Install NVIDIA libraries? [y/N]
                        N # On certain systems, modifying any existing NVIDIA libraries can break things!
                        y # If you are confident about your system dependencies, you may choose to proceed with the installation of NVIDIA libraries.
user@host-pc:~$ cd ..

user@host-pc:~$ mkdir -p autoware_map

user@host-pc:~$ gdown -O ~/Autoware_WS/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'

user@host-pc:~$ unzip -d ~/Autoware_WS/autoware_map ~/Autoware_WS/autoware_map/sample-map-planning.zip

user@host-pc:~$ docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda

user@host-pc:~$ rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/Autoware_WS/autoware_docker --volume $HOME/Autoware_WS/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

## Docker Container:

```bash
user@container-id:~$ cd Autoware_WS/autoware_docker/

user@container-id:~$ mkdir src

user@container-id:~$ vcs import src < autoware.repos

user@container-id:~$ sudo apt update

user@container-id:~$ rosdep update

user@container-id:~$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

user@container-id:~$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Planning Simulation:
```bash
user@host-pc:~$ rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/Autoware_WS/autoware_docker --volume $HOME/Autoware_WS/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda

user@container-id:~$ source ~/Autoware_WS/autoware_docker/install/setup.bash

user@container-id:~$ ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Autoware_WS/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
