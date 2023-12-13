# Autoware Docker Installation Guide

### From Host Computer:

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
                        N
user@host-pc:~$ cd ..

user@host-pc:~$ mkdir -p autoware_map

user@host-pc:~$ docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda

user@host-pc:~$ rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/Autoware_WS/autoware_docker --volume $HOME/Autoware_WS/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

### Within Docker Container:

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
