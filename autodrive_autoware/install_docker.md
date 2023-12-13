# Autoware Docker Installation Guide

![Autoware](https://user-images.githubusercontent.com/63835446/158918717-58d6deaf-93fb-47f9-891d-e242b02cba7b.png)

This documentation covers `Docker Installation of Autoware` and testing the installation with `Autoware Planning Simulation`.

> **Note 1**: Installation was tested with Host-PC running Ubuntu 20.04 (with ROS 1 Noetic, ROS 2 Foxy & ROS 2 Galactic) and Docker-Container running Ubuntu 22.04 (with ROS 2 Humble).

> **Note 2**: A dedicated `Autoware_WS` was created on Host-PC to organize `autoware_maps`, `autoware_docker` installation (with ROS 2 Humble) and `autoware_local` installation (with ROS 2 Galactic).

> **Note 3**: Pay close attention to `user@host-pc` (when executing commands from Host-PC) and `container-id` (when executing commands from Docker-Container).

## Official Documentation:

- For Autoware's general documentation, see [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/).

- For detailed documents of Autoware Universe components, see [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/).

- For the official Autoware installation guide, see [Autoware Installation Guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/).
  - [Local (Source) Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
  - [Docker Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)

- For the official Autoware planning simulation guide, see [Planning Simulation Documentation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/).

- For the official Docker installation guide, see [Get Docker](https://docs.docker.com/get-docker/), and particularly for Ubuntu, see [Docker Ubuntu Installation](https://docs.docker.com/desktop/install/ubuntu/).

---

## Set Up Autoware Development Environment:

1. Create a dedicated workspace for Autoware called `Autoware_WS` on Host-PC to organize different Autoware installations, maps, data, etc and move to the directory.
```bash
user@host-pc:~$ mkdir -p Autoware_WS
user@host-pc:~$ cd Autoware_WS
```

2. Clone the [`autowarefoundation/autoware`](https://github.com/autowarefoundation/autoware.git) repository into `Autoware_WS`, rename it to `autoware_docker` (to differentiate it from `autoware_local` installation, if any) and move to the directory.
```bash
user@host-pc:~$ git clone https://github.com/autowarefoundation/autoware.git
user@host-pc:~$ sudo mv $HOME/Autoware_WS/autoware $HOME/Autoware_WS/autoware_docker
user@host-pc:~$ cd autoware_docker
```

3. Install the Required Dependencies (Autoware uses [Ansible](https://www.ansible.com/) scripts to automate dependency and configuration management).
```bash
user@host-pc:~$ ./setup-dev-env.sh docker # --no-nvidia --no-cuda-drivers (for installation without NVIDIA libraries & CUDA drivers)
user@host-pc:~$ # Setting up the build environment can take up to 1 hour.
                        # >  Are you sure you want to run setup? [y/N]
                        y
user@host-pc:~$ # [Warning] Some Autoware components depend on the CUDA, cuDNN and TensorRT NVIDIA libraries which have end-user license agreements that should be reviewed before installation.
                        # Install NVIDIA libraries? [y/N]
                        N # On certain systems, modifying any existing NVIDIA libraries can break things!
                        y # If you are confident about your system dependencies, you may choose to proceed with the installation of NVIDIA libraries.
```



```bash
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
