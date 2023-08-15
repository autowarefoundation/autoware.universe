# Autoware Installation on Jetson Xavier NX and F1tenth Recordreplay Demo

This tutorial uses ros galactic due to the fact the Nvidia Jetson only supports Ubuntu 20.04 or below. To natively build and run autoware without using docker, galactic instead of humble is used to increase system compatibility.



## Flash Jetpack 5.1.1 to Jetson Xavier NX

1. Follow [Jetpack 5.1.1 Documentation](https://developer.nvidia.com/embedded/jetpack-sdk-511) to flash Jetpack 5.1.1 onto the Jetson NX. You may install via the `SD Card Image Method` or the [NVIDIA SDK Manager Method](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html) described in the documentation. 

2. If you'd like to directly flash the OS onto the `NVMe SSD drive` instead of the `SD card`, and you have an x86 host machine running `Ubuntu 20.04` or `Ubuntu 18.04`, it is recommended that you use the NVIDIA SDK Manager as it allows you to directly flash to the NVMe SSD drive. If you have trouble flashing the Jetpack, you can put the Jetson into `Force Recovery Mode` by using a jumper to connect PINs #9 and #10 of the connector J50 before powering up the Jetson.

3. Once the Jetpack is successfully installed, bootup the system and the Ubuntu desktop environment should launch

## Set up Autoware development environment

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. If you are installing Autoware for the first time, you can automatically install the dependencies by using the provided Ansible script. 

   ```bash
   ./setup-dev-env.sh
   ```

   IMPORTANT: During installation, when asked whether to `Install Cuda Driver?`, enter `N` as the cuda driver is already installed with the Jetpack. If force the cuda driver installation here, it can mess up the kernal and cause error at bootup. You will need to reflash the Jetpack

3. Under the `autoware` folder, go to the auto.repos file and change the version of `universe/autoware.universe` from `main` to `f1tenth_galactic`


## Set up Autoware workspace

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

3. Create swapfile.

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

   Ignore the `stderr` warnings during the build. It will take 5-6 hours to build the workspace on the Jetson NX
   

## Install f1tenth_gym simulator dependencies
The f1tenth_gym_ros simulator is used in this case, click [here](https://github.com/f1tenth/f1tenth_gym_ros) for details.

   ```bash
   cd autoware/src/universe/autoware.universe/f1tenth/f1tenth_gym_ros/f1tenth_gym
   pip3 install -e .
   ```

## How to create a map
[![YouTube Demo](https://img.youtube.com/vi/wANpcMwWCrg/0.jpg)](https://www.youtube.com/watch?v=wANpcMwWCrg)

This part assumes that you have a fully built and properly tuned f1tenth car. For instructions on how to configure an f1tenth car, see [f1tenth_system](https://github.com/autowarefoundation/autoware.universe/tree/f1tenth_galactic/f1tenth/f1tenth_system).

On your f1tenth car, install the slamtoolbox 

   ```bash
   sudo apt install ros-galactic-slam-toolbox
   ```

Start the f1tenth system

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

Start the slamtoolbox

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/autoware/src/universe/autoware.universe/f1tenth/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```

Launch rviz2, Add __/map__ by topic. Add __/graph_visualization__ by topic. On top left corner of rviz, panels – add new panel – add SlamToolBoxPlugin panel. Once you’re done mapping, save the map using the plugin. You can give it a name in the text box next to Save Map. Map will be saved in whichever directory you ran slam_toolbox.

### Create a map without an f1tenth car

If you do not have an f1tenth car, You can draw your own map and save as .png files. Make sure you set the corresponding .yaml file correctly. You can also use the map provided in the f1tenth simulation folder under /map directory.

### Change map in the f1tenth simulator

Navigate to /home/autoware/src/universe/autoware.universe/f1tenth/f1tenth_gym_ros/config. In sim.yaml, change the map file path.

## How to record a trajectory (simulation)

[![YouTube Demo](https://img.youtube.com/vi/iQ3Bsct1q-g/0.jpg)](https://www.youtube.com/watch?v=iQ3Bsct1q-g)

Launch the f1tenth gym simulator, recordreplay node, and trajectory follower

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth demo_launch.py
```

Launch the keyboard teleop tool

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Record a trajectory and save at your preferred path. To stop recording, Ctrl + C and your path will be automatically saved.

```(bash)
# Terminal 3
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```

## How to replay a trajectory (simulation)

Launch the f1tenth gym simulator, recordreplay node, and trajectory follower

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth demo_launch.py
```

Replay a trajectory from your previously saved file

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```

## How to record a trajectory (real car)

[![YouTube Demo](https://img.youtube.com/vi/91juU3qQ08M/0.jpg)](https://www.youtube.com/watch?v=91juU3qQ08M)

Launch the f1tenth_system, recordreplay_planner, and trajectory follower

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth realcar_launch.py
```

Launch the particle filter for localization. You need the library range_libc to utilize the GPU. For instructions on setup, see [particle_filter](https://github.com/autowarefoundation/autoware.universe/tree/f1tenth_galactic/f1tenth/particle_filter).

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch particle_filter localize_launch.py
```

Record a trajectory and save at your preferred path. To stop recording, Ctrl + C and your path will be automatically saved.

```(bash)
# Terminal 3
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```


## How to replay a trajectory (real car)

[![YouTube Demo](https://img.youtube.com/vi/VPEcjZRnqL8/0.jpg)](https://www.youtube.com/watch?v=VPEcjZRnqL8)

Launch the f1tenth_system, recordreplay_planner, and trajectory follower

```(bash)
# Terminal 1
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth realcar_launch.py
```

Launch the particle filter for localization. You need the library range_libc to utilize the GPU. For instructions on setup, see [particle_filter](https://github.com/autowarefoundation/autoware.universe/tree/f1tenth_galactic/f1tenth/particle_filter).

```(bash)
# Terminal 2
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 launch particle_filter localize_launch.py
```

Replay a trajectory from your previously saved file

```(bash)
# Terminal 3
source /opt/ros/galactic/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```