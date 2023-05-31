# Autoware F1tenth Recordreplay Demo

## How to set up autoware development environment

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. If you are installing autoware for the first time, you can automatically install the dependencies by using the provided Ansible script. Be careful this script will change some of your drivers and system settings. If you are not sure about this, you can install the dependencies manually. Check the [autoware installtion](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/) page for more details.

   ```bash
   ./setup-dev-env.sh
   ```

3. Go to the auto.repos file and change the version of `universe/autoware.universe` from `main` to `f1tenth_galactic`

## How to set up autoware workspace

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
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

3. Build the workspace.

   autoware uses [colcon](https://github.com/colcon) to build workspaces.
   For more advanced options, refer to the [documentation](https://colcon.readthedocs.io/).

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
   
## Install f1tenth_gym simulator dependencies
The f1tenth_gym_ros simulator is used in this case, click [here](https://github.com/f1tenth/f1tenth_gym_ros) for details.

   ```bash
   cd autoware/src/universe/autoware.universe/f1tenth/f1tenth_gym_ros/f1tenth_gym
   pip3 install -e .
   ```

## How to create a map
Click [here](https://drive.google.com/file/d/15PViYjO-CKy2uvMqojIPj7BqdnMEidws/view?usp=share_link) to watch a demonstration video. 

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

Click [here](https://drive.google.com/file/d/1AkPndsR42yFxAHseP_JW0zUfrvYIeyNb/view?usp=share_link) to watch a demonstration video. 

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