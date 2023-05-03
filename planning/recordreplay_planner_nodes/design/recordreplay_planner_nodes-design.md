# RecordReplay planner nodes {#recordreplay-planner-nodes}

# Purpose / Use cases

This package provides a ROS2 node for the computational package `recordreplay_planner`. This is done to
separate the computation part from the ROS2-specific part. See the computational part for a rationale of what
the functionality is to be used for.

Sometimes it is desired to loop over the recorded trajectory when replaying. This can be done by setting
`loop_trajectory` to `True` and use an appropriate `loop_max_gap_m` to prevent unwanted looping when the start and end
point of the trajectory are too far away.

# Design

This is a wrapper around `recordreplay_planner`. Its behavior can be controlled via actions. It can record
the ego state of the vehicle and play back a set of recorded states at a later time.

## Inputs / Outputs / API

Actions:

This node uses two actions to control its behavior:

- `RecordTrajectory.action` is used to record a trajectory. It runs until canceled. While the action is
  running, the node subscribes to a `Odometry.msg` topic by a provided name and records all
  states that are published on that topic.
- `ReplayTrajectory.action` is used to replay a trajectory. It runs until canceled. While the action is
  running, the node subscribes to the same `Odometry.msg` topic as when recording. When messages
  are published on that topic, the node publishes a trajectory starting approximately at that point (see the
  `recordreplay_planner` design documentation on how that point is determined).

The actions are defined in `autoware_auto_planning_msgs` and `autoware_auto_perception_msgs`.

Inputs:

- `autoware_auto_vehicle_msgs/msg/Odometry` is the state used as recorded points for replay, and also to prune starting point of replay trajectory

Outputs:

- `autoware_auto_planning_msgs/msg/Trajectory` is the trajectory that gets published. If `enable_object_collision_estimator`, then it is the trajectory after modification.

## Complexity

See `recordreplay_planner`.

# Security considerations

TBD by a security specialist.
