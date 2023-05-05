# Running the Autoware Trajectory Follower with F1Tenth

clone the autoware.universe repository

`https://github.com/autowarefoundation/autoware.universe.git`



ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback

ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
