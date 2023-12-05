### From Host Computer:

```bash
csamak@alienware-x15:~$ mkdir -p Autoware_WS

csamak@alienware-x15:~$ cd Autoware_WS

csamak@alienware-x15:~$ git clone https://github.com/autowarefoundation/autoware.git

csamak@alienware-x15:~$ sudo mv $HOME/Autoware_WS/autoware $HOME/Autoware_WS/autoware_docker

csamak@alienware-x15:~$ cd autoware_docker

csamak@alienware-x15:~$ ./setup-dev-env.sh docker

csamak@alienware-x15:~$ # Setting up the build environment can take up to 1 hour.
                        # >  Are you sure you want to run setup? [y/N]
                        y

csamak@alienware-x15:~$ # [Warning] Some Autoware components depend on the CUDA, cuDNN and TensorRT NVIDIA libraries which have end-user license agreements that should be reviewed before installation.
                        # Install NVIDIA libraries? [y/N]
                        N

csamak@alienware-x15:~$ cd ..

csamak@alienware-x15:~$ mkdir -p autoware_map

csamak@alienware-x15:~$ docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda

csamak@alienware-x15:~$ rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/Autoware_WS/autoware_docker --volume $HOME/Autoware_WS/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

### Within Docker Container:

```bash
csamak@35c49fc76477:~$ cd Autoware_WS/autoware_docker/

csamak@35c49fc76477:~$ mkdir src

csamak@35c49fc76477:~$ vcs import src < autoware.repos

csamak@35c49fc76477:~$ sudo apt update

csamak@35c49fc76477:~$ rosdep update

csamak@35c49fc76477:~$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

csamak@35c49fc76477:~$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Planning Simulation:
```bash
alienware-x15:~$ rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/Autoware_WS/autoware_docker --volume $HOME/Autoware_WS/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda

csamak@35c49fc76477:~$ source ~/Autoware_WS/autoware_docker/install/setup.bash

csamak@35c49fc76477:~$ ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Autoware_WS/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```