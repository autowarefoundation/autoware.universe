# F1Tenth RecordReplay Trajectory Demo

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

ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback

ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback

```

```
