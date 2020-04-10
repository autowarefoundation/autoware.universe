# autoware.proj

meta-repository for Tier IV's projects

![autoware](https://user-images.githubusercontent.com/8327598/69472442-cca50b00-0ded-11ea-9da0-9e2302aa1061.png)

## Requirements

### Hardware

- x86 CPU (8 or more cores)
- 16 GB or more of memory
- Nvidia GPU (4GB or more of memory)

### Software

- Ubuntu 18.04
- Nvidia driver

If cuda or tensorRT is already installed, it is recommended to remove it.

## How to setup

1. Set up the repository

If ROS hasn't been installed yet in PC, at first run commands of 1.2 ~ 1.4, described in [ROS wiki](http://wiki.ros.org/melodic/Installation/Ubuntu).

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

Set up the repository

```sh
sudo apt install -y python3-vcstool
git clone git@github.com:tier4/autoware.proj.git
cd autoware.proj
mkdir -p src
vcs import src < autoware.proj.repos
```

2. Run the setup script

```sh
./setup_ubuntu18.04.sh
```

In this step, the following software are installed.  
Please confirm their licenses before using them.

- [osqp](https://github.com/oxfordcontrol/osqp/blob/master/LICENSE)
- [ROS Melodic](https://github.com/ros/ros/blob/noetic-devel/LICENSE)
- [CUDA 10.2](https://docs.nvidia.com/cuda/eula/index.html)
- [cuDNN 7](https://docs.nvidia.com/deeplearning/sdk/cudnn-sla/index.html)
- [TensorRT 7](https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html)

3. Build the source

```sh
source ~/.bashrc
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests
```

## How to configure

### Set hardware configuration

Prepare launch files and vehicle_description according to the sensor configuration of your hardware.  
The following are the samples.

- [sensing.launch](https://github.com/tier4/autoware_launcher/blob/master/sensing_launch/launch/sensing.launch)
- [vehicle_description](https://github.com/tier4/Autoware-T4B/tree/master/vehicle/vehicle_description)

## How to run

### Supported Simulations

![sim](https://user-images.githubusercontent.com/8327598/79709776-0bd47b00-82fe-11ea-872e-d94ef25bc3bf.png)

### Quick Start

#### Rosbag

1. Download sample map from [here](https://drive.google.com/open?id=1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk).

2. Download sample rosbag from [here](https://drive.google.com/open?id=1BFcNjIBUVKwupPByATYczv2X4qZtdAeD).
3. Launch Autoware

```sh
cd autoware.proj
source install/setup.bash
roslaunch autoware_launch autoware.launch map_path:=[path] vehicle_model:=[vehicle_model] sensor_model:=[sensor_model] rosbag:=true
```

4. Play rosbag

```sh
rosbag play --clock [rosbag file] -r 0.2
```

##### Note

- sample map : © 2020 TierIV inc.
- rosbag : © 2020 TierIV inc.
  - Image data are removed due to privacy concerns.
    - Cannot run traffic light recognition
    - Decreased accuracy of object detection

#### Planning Simulator

1. Download sample map from [here](https://drive.google.com/open?id=197kgRfSomZzaSbRrjWTx614le2qN-oxx).

2. Launch Autoware

```sh
cd autoware.proj
source install/setup.bash
roslaunch autoware_launch planning_simulator.launch map_path:=[path] vehicle_model:=[vehicle_model] sensor_model:=[sensor_model]
```

3. Set initial pose
4. Set goal pose
5. Push engage button.
   [autoware_web_controller](http://localhost:8085/autoware_web_controller/index.html)

##### Note

- sample map : © 2020 TierIV inc.

#### Tutorial in detail

See [here](./docs/SimulationTutorial.md). for more information.

#### Real car

1. Launch Autoware

```sh
cd autoware.proj
source install/setup.bash
roslaunch autoware_launch autoware.launch map_path:=[path] vehicle_model:=[vehicle_model] sensor_model:=[sensor_model]
```

## References

### Videos

- [Scenario demo](https://youtu.be/kn2bIU_g0oY)
- [Obstacle avoidance in the same lane](https://youtu.be/s_4fBDixFJc)
- [Obstacle avoidance by lane change](https://youtu.be/SCIceXW9sqM)
- [Object recognition](https://youtu.be/uhhMIxe1zxQ)
- [Auto parking](https://youtu.be/e9R0F0ZJbWE)
- [360° FOV perception(Camera Lidar Fuison)](https://youtu.be/whzx-2RkVBA)
- [Robustness of localization](https://youtu.be/ydPxWB2jVnM)

### Credits

- [Neural Network Weight Files](./docs/Credits.md)

### For Tier IV Developpers

 - [Map data information (Confluence)](https://tier4.atlassian.net/wiki/spaces/KB4FAE/pages/33358143)
