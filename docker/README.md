# Dockerfile for autoware.proj

This directory contains files to run autoware.proj on Docker.

## Build

Just run `build.sh` in this directory. This takes some time.

```sh
./docker/build.sh
```

## Run

### Prerequisites

- [rocker](https://github.com/osrf/rocker)

  To enable GUI applications on Docker Containers, this script uses `rocker`.
  Please see [here](http://wiki.ros.org/docker/Tutorials/GUI) for more details.

  ```sh
  sudo apt-get install python3-rocker
  ```

### Examples

#### How to do a rosbag Simulation

```sh
# Build your docker image
cd ~/
git clone git@github.com:tier4/autoware.proj.git
~/autoware.proj/docker/build.sh

# Create and move to a workspace
mkdir ~/rosbag_simulation
cd ~/rosbag_simulation

# Download a sample map
gdown --id 1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk
unzip rosbag_sample_map.zip -d map

# Download a sample rosbag file
mkdir sample
cd sample
gdown --id 1wLWyOlfH_-k4VYBgae1KAFlKdwJnH_si

# Accept X-access from localhost
xhost +local:

# Run Autoware in the simulation mode
rocker --nvidia --x11 --user --volume ~/rosbag_simulation -- autoware:pre-built
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/rosbag_simulation/map vehicle_model:=lexus sensor_model:=aip_xx1

# Play the sample rosbag file in another terminal
rocker --nvidia --x11 --user --volume ~/rosbag_simulation -- autoware:pre-built
ros2 bag play ~/rosbag_simulation/sample/sample.625-2.bag2_0.db3 -r 0.2
```
