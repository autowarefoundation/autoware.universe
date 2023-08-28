# Autoware Installation on Jetson Xavier NX and F1tenth Recordreplay Demo

This tutorial provides step-by-step instructions for installing and setting up the Autoware development environment on the F1tenth race car. The Autoware installation process in this branch is modified from the main one to adapt to the Jetson Xavier NX hardware and software systems. One major difference of this Autoware environment is that it runs on `ROS2 galactic` instead of `ROS2 humble` due to the fact that the NVIDIA Jetson currently only supports Ubuntu 20.04 or below. To natively build and run autoware without using docker, galactic is used to increase system compatibility. The original [Autoware installation documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) from main branch, and the [F1tenth build documentation](https://f1tenth.readthedocs.io/en/foxy_test/index.html)(running ROS2 foxy) are here for your reference.

This repo also includes a F1tenth Recordreplay demo. This demo allows the user to first build a map, record a trajectory by manually driving the F1tenth race car, and then perform trajectory following in both `real-world`(testing in progress) and in the `F1tenth gym simulator` running the Autoware framework. Instructions for installing the F1tenth gym simulator are provided. The approximate time investment is based on running Jetson Xavier NX on 20W 6core power mode.

## Flash JetPack 5.1.1 (rev. 1) to Jetson Xavier NX
(Approximate Time Investment: 1-1.5 hours)

There are multiple ways to install JetPack on a Jetson as described in [Jetpack 5.1.1 Documentation](https://developer.nvidia.com/embedded/jetpack-sdk-511). The recommended ways to install are via the `NVIDIA SDK Manager Method` or the `SD Card Image Method`. This repo was tested on JetPack 5.1.1. Other JetPack versions may also work but have not yet been tested.

### NVIDIA SDK Manager Method:
(requires a Linux host computer running Ubuntu Linux x64 version 20.04 or 18.04 with ~25GB(for target components)+15GB(for host components, optional) of space)

This method you will first install `NVIDIA SDK Manager` on your host machine, connect the host machine to the Jetson Xavier NX via a micro-USB cable, download all of the necessary JetPack components using the SDK Manager, and then flash the JetPack to the target Jetson Xavier NX. This method allows you to directly flash the JetPack to the `SD Card` or to the `NVME SSD drive` on the race car Jetson. You may need to create an NVIDIA account to download the NVIDIA SDK manager.

1. Download and install [SDK Manager](https://developer.nvidia.com/sdk-manager) on your host machine.

2. Follow the steps at [Install Jetson Software with SDK Manager](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html). Select JetPack version 5.1.1 (rev. 1). The target hardware will be the Jetson Xavier NX.

3. If you have trouble flashing the JetPack, you can put the Jetson into `Force Recovery Mode` by using a jumper to connect `PINs #9 and #10` of the connector J50 before powering up the Jetson.


### SD Card Image Method:
(requires a computer with Internet connection and the ability to read and write SD cards)

1. Download [JetPack 5.1.1](https://developer.nvidia.com/downloads/embedded/l4t/r35_release_v3.1/sd_card_b49/jp511-xnx-sd-card-image.zip/)

2. If you have not previously run a JetPack 5.x release on your Jetson Xavier NX Developer kit, you must first update its QSPI before using this JetPack 5.x SD Card image. See the [SD Card Image Method](https://developer.nvidia.com/embedded/jetpack-sdk-511) section for more information.

2. Follow the steps at [Jetson Xavier NX Developer Kit - Get Started](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit#prepare) to write the Jetpack to the microSD card.

3. Insert your microSD card to the Jetson.

Once the JetPack is successfully flashed to the Jetson NX, boot the system and the Ubuntu desktop environment should launch


## Install ROS2 galactic 
(Approximate Time Investment: 0.5 hour)

1. Follow the [ROS2 instructions](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) to install ROS2 galactic


## Set up Autoware development environment 
(Approximate Time Investment: 0.5 hour)

1. Clone the `galactic` branch of `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone -b galactic https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. If you are installing Autoware for the first time, you can automatically install the dependencies by using the provided Ansible script. 

   ```bash
   ./setup-dev-env.sh --no-nvidia --no-cuda-drivers
   ```

   The NVIDIA library and cuda driver installation are disabled as they are already installed with the JetPack. If you force the cuda driver installation here, it can mess up the kernel and cause errors at bootup. You will need to reflash the JetPack if this happens

3. Under the `autoware` folder, go to the `autoware.repos` file and change the version of `universe/autoware.universe` from `galactic` to `f1tenth_galactic`


## Set up Autoware workspace 
(Approximate Time Investment: 6-7 hours)

1. Create the `src` directory and clone repositories into it.

   autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces.

   ```bash
   cd autoware
   mkdir src
   vcs import src < autoware.repos
   ```

2. Install dependent ROS packages.

   ```bash
   source /opt/ros/galactic/setup.bash
   rosdep update --include-eol-distros
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r
   ```

   Ignore the `Invalid version` errors during rosdep installation

3. Create swapfile. Originally from Autoware [troubleshooting section](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/#build-issues)

   Building Autoware requires a lot of memory. Jetson NX can crash during a build because of insufficient memory. To avoid this problem, 16-32GB of swap should be configured.

   Optional: Check the current swapfile
   ```bash
   free -h
   ```

   Remove the current swapfile
   ```bash
   sudo swapoff /swapfile
   sudo rm /swapfile`
   ```
   
   Create a new swapfile
   ```bash
   sudo fallocate -l 32G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

   Optional: Check if the change is reflected
   ```bash
   free -h
   ```

4. Build the workspace.

   autoware uses [colcon](https://github.com/colcon) to build workspaces.
   For more advanced options, refer to the [documentation](https://colcon.readthedocs.io/).

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   Ignore the `stderr` warnings during the build.
   

## Install f1tenth_gym simulator dependencies
(Approximate Time Investment: 10 minutes)

Install `onnx` and roll back `setuptools` to version 65.5.0
   ```bash
   pip3 install onnx setuptools==65.5.0
   ```
The f1tenth_gym_ros simulator is used in this case, click [here](https://github.com/f1tenth/f1tenth_gym_ros) for details.

   ```bash
   cd autoware/src/universe/autoware.universe/f1tenth/f1tenth_gym_ros/f1tenth_gym
   pip3 install -e .
   ```

# F1tenth Recordreplay Demo

## How to create a map

This part assumes that you have a fully built and properly tuned f1tenth car. For instructions on how to configure an f1tenth car, see [f1tenth_system](https://github.com/autowarefoundation/autoware.universe/tree/f1tenth_galactic/f1tenth/f1tenth_system).

On your f1tenth car, install the slamtoolbox 

   ```bash
   sudo apt install ros-galactic-slam-toolbox
   ```

1. Start the f1tenth system

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

2. Start the slamtoolbox

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/autoware/src/universe/autoware.universe/f1tenth/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```

3. Launch RViz2, Add __/map__ by topic. Add __/graph_visualization__ by topic. On the top left corner of rviz, panels – add new panel – add SlamToolBoxPlugin panel. Once you’re done mapping, save the map using the plugin. You can give it a name in the text box next to Save Map. Map will be saved in whichever directory you run slam_toolbox.

### Create a map without an f1tenth race car

If you do not have an f1tenth car, You can draw your own map and save as .png files. Make sure you set the corresponding .yaml file correctly. You can also use the map provided in the f1tenth simulation folder under /map directory.

### Change map in the f1tenth simulator

Navigate to /home/autoware-f1/autoware/install/f1tenth_gym_ros/share/f1tenth_gym_ros/config. In `sim.yaml`, update the map file path.

## How to record a trajectory (simulation)

1. Use the `demo_launch` launch file to launch `gym_bridge`, `recordreplay_planner`, and `trajectory_follower_f1tenth` nodes. 

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth demo_launch.py
```

Rviz2 should launch automatically with the target map loaded (black markers). After a short peroid of time the simulated Lidar data (markers with color) should be overlaid on top of the map indicating the simulation is running correctly. It can take up to `5 minutes` for the Lidar data to show up if the simulator is launched for the first time. You may adjust the camera angle using your mouse's left and right buttons and scroll wheel.
![RViz image](f1tenth_sim.jpg)

2. Launch the `teleop_twist_keyboard` node for keyboard tele-operation. Focus on (select) this terminal and use `U`, `I`, `O` keys to manually control the f1tenth car in the simulation.  Use `Q` and `Z` keys to increase and decrease the speed.

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. Record a trajectory and save at your preferred path. To stop recording, `Ctrl + C` and your path will be automatically saved. 
The default path for the recording is set to `"/tmp/path"`. This recording will be automatically erased after system reboot. 

```(bash)
# Terminal 3
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```

## How to replay a trajectory (simulation)

1. Use the `demo_launch` launch file to launch `gym_bridge`, `recordreplay_planner`, and `trajectory_follower_f1tenth` nodes if they are not currently running.

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth demo_launch.py
```

2. Replay a trajectory from your previously saved file. You can use the `2D Pose Estimate` tool in RViz2 anytime to reset the car's pose.

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```

## How to record a trajectory (real car)

[![YouTube Demo](https://img.youtube.com/vi/91juU3qQ08M/0.jpg)](https://www.youtube.com/watch?v=91juU3qQ08M)

1. Use the `realcar_launch` launch file to launch the `f1tenth_stack`, `recordreplay_planner`, and `trajectory_follower_f1tenth` nodes.

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth realcar_launch.py
```

2. Launch the `particle_filter` node for localization. You need the library range_libc to utilize the GPU. For instructions on setup, see [particle_filter](https://github.com/autowarefoundation/autoware.universe/tree/f1tenth_galactic/f1tenth/particle_filter).

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch particle_filter localize_launch.py
```

3. Record a trajectory and save at your preferred path. To stop recording, `Ctrl + C` and your path will be automatically saved. 
The default path for the recording is is set to `"/tmp/path"`. This recording will be automatically erased after system reboot.

```(bash)
# Terminal 3
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```

## How to replay a trajectory (real car)

1. Use the `realcar_launch` launch file to launch `f1tenth_stack`, `recordreplay_planner`, and `trajectory_follower_f1tenth` nodes.

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth realcar_launch.py
```

2. Launch the `particle_filter` node for localization. You need the library range_libc to utilize the GPU. For instructions on setup, see [particle_filter](https://github.com/autowarefoundation/autoware.universe/tree/f1tenth_galactic/f1tenth/particle_filter).

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch particle_filter localize_launch.py
```

3. Replay a trajectory from your previously saved file

```(bash)
# Terminal 3
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```