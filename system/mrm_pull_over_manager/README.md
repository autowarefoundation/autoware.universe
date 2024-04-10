# mrm_pull_over_manager

## Purpose

This node sends candidate pull over poses to emergency_goal_manager according to the pull over MRM request.

## Inner-workings / Algorithms

TBD.

### State Transitions

TBD.

## Inputs / Outputs

### Input

| Name                            | Type                                           | Description                      |
| ------------------------------- | ---------------------------------------------- | -------------------------------- |
| `~/input/odometry`              | `nav_msgs::msg::Odometry`                      | To get current pose and velocity |
| `~/input/route`                 | `autoware_planning_msgs::msg::LaneletRoute`    | To get current route             |
| `~/input/lanelet_map`           | `autoware_auto_mapping_msgs::msg::HADMapBin`   | To calculate pull over points    |
| `~/input/trajectory`            | `autoware_auto_planning_msgs::msg::Trajectory` | To calculate pull over points    |
| `~/input/mrm/pull_over/operate` | `tier4_system_msgs::srv::OperateMrm`           | MRM pull over request            |

### Output

| Name                                         | Type                                                 | Description               |
| -------------------------------------------- | ---------------------------------------------------- | ------------------------- |
| `~/output/mrm/pull_over/emergency_goals`     | `tier4_system_msgs::msg::EmergencyGoalsStamped`      | Candidate pull over poses |
| `~/output/mrm/pull_over/status`              | `tier4_system_msgs::msg::MrmBehaviorStatus`          | Pull over MRM status      |
| `~/output/mrm/pull_over/goals_clear_command` | `tier4_system_msgs::msg::EmergencyGoalsClearCommand` | Clear command             |

## Parameters

### Node Parameters

| Name          | Type   | Description                     | Default |
| ------------- | ------ | ------------------------------- | ------- |
| `update_rate` | double | Timer callback update rate [Hz] | 10.0    |

### Core Parameters

| Name                      | Type   | Description                                            | Default |
| ------------------------- | ------ | ------------------------------------------------------ | ------- |
| `max_goal_pose_num`       | int    | Maximum number of candidate goal poses [-]             | 3       |
| `yaw_deviation_threshold` | double | Yaw deviation threshold for candidate goal poses [rad] | 0.5     |
| `margin_time_to_goal`     | double | Arrival time threshold for candidate goal poses [sec]  | 10.0    |

## Assumptions / Known limits

TBD.
