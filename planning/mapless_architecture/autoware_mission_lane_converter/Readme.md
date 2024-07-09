# Mission Lane Converter

Converts the selected mission lane to an autoware trajectory.

## Input topics

| Name                                         | Type                                             | Description   |
| -------------------------------------------- | ------------------------------------------------ | ------------- |
| `mission_lane_converter/input/odometry`      | nav_msgs::msg::Odometry                          | odometry      |
| `mission_lane_converter/input/mission_lanes` | autoware_planning_msgs::msg::MissionLanesStamped | mission lanes |

## Output topics

| Name                                              | Type                                    | Description       |
| ------------------------------------------------- | --------------------------------------- | ----------------- |
| `mission_lane_converter/output/trajectory`        | autoware_planning_msgs::msg::Trajectory | trajectory        |
| `mission_lane_converter/output/global_trajectory` | autoware_planning_msgs::msg::Trajectory | global trajectory |
| `mission_lane_converter/output/path`              | autoware_planning_msgs::msg::Path       | path              |
| `mission_lane_converter/output/global_path`       | autoware_planning_msgs::msg::Path       | global path       |

## Node parameters

| Parameter      | Type  | Description  |
| -------------- | ----- | ------------ |
| `target_speed` | float | target speed |
