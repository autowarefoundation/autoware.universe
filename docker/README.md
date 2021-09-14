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
# NOTE: Autoware runs with root privileges since NVIDIA driver has to be enabled
rocker --nvidia --x11 --user --privileged --volume ~/rosbag_simulation -- autoware:pre-built
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/rosbag_simulation/map vehicle_model:=lexus sensor_model:=aip_xx1

# Play the sample rosbag file in another terminal
rocker --nvidia --x11 --user --volume ~/rosbag_simulation -- autoware:pre-built
ros2 bag play ~/rosbag_simulation/sample/sample.625-2.bag2_0.db3 -r 0.2
```

### For Autoware development

In Autoware development, it is often better to mount a local project directory and do the development on a local environment rather than developing in a container. The reasons are because:

- You can use your favorite editor and do your development as always you do in the local environment. You don't need to install your editor to the container.
- Existing changes in the local files will be reflected to the container. Also, changes are preserved even after the container is destroyed.

You can realize this by following the procedure below.

#### 1. Do your modification to the project

Open a terminal and go to the project directory, say `~/autoware.proj`.

```sh
cd ~/autoware.proj/
```

Build Docker images if you haven't done yet.

```sh
./docker/build.sh
```

Import dependencies if you haven't done yet.

```sh
mkdir src
vcs import src < autoware.proj.repos
```

Do your modification to the project just as you want.

#### 2. Launch a container

Open another terminal and launch a container. Note that the command below is slightly different from the example above.
We use `autoware:base` instead of `autoware:pre-built`, and your home directory will be mounted as we specify the `--home` option.

```sh
xhost +local:
rocker --nvidia --x11 --user --home -- autoware:base
```

Or you can manually specify the mount destination

```sh
rocker --nvidia --x11 --user --volume ~/rosbag_simulation --volume ~/autoware.proj:$HOME/autoware.proj -- autoware:base
```

#### 3. Build

In the container, go to the project directory (in this case `~/autoware.proj`) and build.

```sh
cd ~/autoware.proj
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 4. Execute

If you want to launch the logging simulator, run the commands below in the container.

```sh
source install/setup.bash
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/rosbag_simulation/map vehicle_model:=lexus sensor_model:=aip_xx1
```

Open another terminal and play a rosbag file.

```sh
rocker --nvidia --x11 --user --volume ~/rosbag_simulation -- autoware:pre-built
ros2 bag play ~/rosbag_simulation/sample/sample.625-2.bag2_0.db3 -r 0.2
```

NOTE: Autoware-specific message types will be redefined irregularly. Make sure that these two containers have the same message definitions.
