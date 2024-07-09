# Mission Planner

Creates a target lane based on the mission input. The output is forwarded to the converter node.

Here, one can see the target lane:

![target lane](images/Targetlane.png)

The two plots show the point in time, when the lane change was triggered.

## Input topics

| Name                                   | Type                                  | Description |
| -------------------------------------- | ------------------------------------- | ----------- |
| `mission_planner_node/input/local_map` | autoware_planning_msgs::msg::LocalMap | local map   |
| `mission_planner/input/mission`        | autoware_planning_msgs::msg::Mission  | mission     |
| `mission_planner/input/state_estimate` | nav_msgs::msg::Odometry               | odometry    |

## Output topics

| Name                                                | Type                                             | Description   |
| --------------------------------------------------- | ------------------------------------------------ | ------------- |
| `mission_planner_node/output/mission_lanes_stamped` | autoware_planning_msgs::msg::MissionLanesStamped | mission lanes |

## Node parameters

| Parameter                          | Type  | Description                                                                                                  |
| ---------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------ |
| `distance_to_centerline_threshold` | float | threshold to determine if lane change mission was successful (if ego is in proximity to the goal centerline) |
| `projection_distance_on_goallane`  | float | projection distance of goal point                                                                            |
| `retrigger_attempts_max`           | int   | number of attempts for triggering a lane change                                                              |
