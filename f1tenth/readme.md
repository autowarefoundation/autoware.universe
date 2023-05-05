# F1Tenth RecordReplay Trajectory Demo

clone the autoware.universe repository

`git clone https://github.com/autowarefoundation/autoware.git`

Create the src directory and clone repositories into it.

Autoware uses `vcstool` to construct workspaces.

````cd autoware
mkdir src
vcs import src < autoware.repos```

`source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO`

`source /opt/ros/galactic/setup.bash`



ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback

ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
````
