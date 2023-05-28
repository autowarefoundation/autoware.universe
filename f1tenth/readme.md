# F1Tenth RecordReplay Trajectory Demo

## How to set up a development environment

1. Clone `autowarefoundation/autoware` and move to the directory.

   ```bash
   git clone https://github.com/autowarefoundation/autoware.git
   cd autoware
   ```

2. Go to the auto.repos file and change the version of `universe/autoware.universe` from `main` to `f1tenth`

## How to set up a workspace

1. Create the `src` directory and clone repositories into it.

   Autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces.

   ```bash
   cd autoware
   mkdir src
   vcs import src < autoware.repos
   ```

2. Install dependent ROS packages.

   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

3. Build the workspace.

   Autoware uses [colcon](https://github.com/colcon) to build workspaces.
   For more advanced options, refer to the [documentation](https://colcon.readthedocs.io/).

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
   

## Install f1tenth_gym simulator dependencies
   ```bash
   cd autoware/src/universe/autoware.universe/f1tenth/f1tenth_gym_ros/f1tenth_gym
   pip3 install -e .
   ```

## How to record a trajectory

Launch the [f1tenth gym simulator](https://github.com/f1tenth/f1tenth_gym_ros), recordreplay node, and trajectory follower (not used)

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
source /opt/ros/humble/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```

## How to replay a trajectory

Launch the [f1tenth gym simulator](https://github.com/f1tenth/f1tenth_gym_ros), recordreplay node, and trajectory follower

```(bash)
# Terminal 1
source /opt/ros/humble/setup.bash
cd autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth demo_launch.py
```

Replay a trajectory from your previously saved file

```(bash)
# Terminal 2
source /opt/ros/humble/setup.bash
cd autoware && . install/setup.bash
ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```

