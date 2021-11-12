Freespace planner {#freespace-planner}
===============

This is the design document for the `freespace_planner` package.

# Purpose / Use cases


This package is a 2D trajectory planner node, which handles static and dynamic obstacles.
This node is based on Hybrid A\* search algorithm implemented in `astar_search` package and uses
custom costmap provider.

# Design

Main purpose of this package is to wrap Hybrid A\* algorithm from `astar_search` and communicate it
with costmap provider and high-level planner, which decides when to request planning.
This package is designed to properly initialize ROS2 communication, initialize trajectory planner and make them cooperate
on action service basis. Operation of the node can be described by the following steps:

1. Freespace planner receives lanelet2 route as request for planner to obtain trajectory in parking space
   and checks it's current state.
   * If currently any planning is taking place, then request is rejected.
   * In other case request is accepted and planner's state changes from `idle` to `planning`.
3. In next step a request for costmap is created and is sent to action server.
   * If costmap request gets rejected then planning is aborted and planner's state is changed into `idle`.
4. Planner checks if returned costmap isn't empty.
   * If costmap is empty then planning ends and planner's state is changed into `idle`.
5. After receiving costmap, planner starts planning with use of `astar_search` package.
   * If costmap response returns failure then planner returns failure and it's state changes to `idle`.
6. Trajectory is published for debug and visualisation purposes.
7. Finally trajectory is sent back with result flag set to successful. Planner's state changes into `idle`.


## Inputs / Outputs / API

### Input

### Output topics

| Name                             | Type                                  | Description                                                 |
| -------------------------------- | ------------------------------------- | ----------------------------------------------------------- |
| `~/output/trajectory`            | autoware_auto_msgs::msg::Trajectory   | whole trajectory for debug purposes                         |
| `~/output/trajectory_pose_array` | geometry_msgs::msg::PoseArray         | trajectory converted into more visualisation-friendly format|

### Action Client

| Name                      | Type                                       | Description                                                 |
| ------------------------- | ------------------------------------------ | ----------------------------------------------------------- |
| `generate_costmap`        | autoware_auto_msgs::action::PlannerCostmap | action client for requesting costmap for requested route    |

### Action Server

| Name                      | Type                                       | Description                                                            |
| ------------------------- | ------------------------------------------ | ---------------------------------------------------------------------- |
| `plan_parking_trajectory` | autoware_auto_msgs::action::PlanTrajectory | action server for calculating trajectory based on obstacles in costmap |

## Configuration

### Node specific parameters

The following parameters are taken from `yaml` parameter file.

| Parameter                     | Type   | Unit | Description                                             |
| ----------------------------- | ------ | ---- | ------------------------------------------------------- |
| `use_back`                    | bool   | -    | whether using backward trajectory                       |
| `only_behind_solutions`       | bool   | -    | whether restricting the solutions to be behind the goal |
| `time_limit`                  | double | ms   | time limit of planning                                  |
| `maximum_turning_radius`      | double | m    | maximum turning radius of robot                         |
| `turning_radius_size`         | double | -    | the number of possible turning radiuses discretization  |
| `theta_size`                  | double | -    | the number of angle's discretization                    |
| `goal_lateral_tolerance`      | double | m    | lateral tolerance of goal pose                          |
| `goal_longitudinal_tolerance` | double | m    | longitudinal tolerance of goal pose                     |
| `goal_angular_tolerance`      | double | rad  | angular tolerance of goal pose                          |
| `curve_weight`                | double | -    | additional cost factor for curve actions                |
| `reverse_weight`              | double | -    | additional cost factor for reverse actions              |
| `obstacle_threshold`          | double | -    | threshold for regarding a certain grid as obstacle      |
| `distance_heuristic_weight`   | double | -    | heuristic weight for estimating node's cost             |

### Vehicle specific parameters

The following parameters are obtained with use of `vehicle_constants_manager` node.

| Parameter                   | Type   | Unit | Description                                             |
| --------------------------- | ------ | ---- | ------------------------------------------------------- |
| `robot_length`              | double | m    | robot length                                            |
| `robot_width`               | double | m    | robot width                                             |
| `cg2back`                   | double | m    | distance between center of gravity and back of the car  |
| `minimum_turning_radius`    | double | m    | minimum turning radius of robot                         |

# Future extensions / Unimplemented parts

* For now, due to difference in required HAD map provider, planner doesn't
  inherit from `TrajectoryPlannerNodeBase` class, since it's interface is insufficient.
  Further development should start from reconsidering interfacing with higher level planners.
* At this point planning as an action doesn't return any feedback to action client,
  because `behavior_planner` doesn't make any use of it.
* Additionally all services should be called sequentially. However rclcpp API doesn't allow making
  sequential service calls, since they might cause deadlock, when calling server inside another service callback.
  This is the main reason why `freespace_planner` and `costmap_generator` communicate with action server.
