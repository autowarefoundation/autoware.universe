# Planning Validator

This `planning_validator` takes validation of the trajectory to prevent unexpected abnormal trajectory from being published. When an invalid trajectory is detected, the `planning_validator` will either "not publish the trajectory" or "publish the latest valid trajectory" according to the selected option. The status of the validation can be seen in the `/diagnostics` and `/validation_status` topics.

## Supported features

The following functions are supported for trajectory validation. Thresholds for the validation can be set by parameters. To turn off a certain validation item individually, set a large value for the associated threshold.

- invalid field (e.g. Inf, Nan)
- trajectory points interval (invalid if any of the distance of trajectory points is too large)
- curvature (invalid if the trajectory has too sharp turns that is not feasible for the given vehicle kinematics)
- relative angle (invalid if the yaw angle changes too fast in the sequence of trajectory points)
- lateral acceleration (invalid if the expected lateral acceleration/deceleration is too large)
- longitudinal acceleration/deceleration (invalid if the acceleration/deceleration in the trajectory point is too large)
- steering angle (invalid if the expected steering value is too large estimated from trajectory curvature)
- steering angle rate (invalid if the expected steering rate value is too large)
- velocity deviation (invalid if the planning velocity is too far from the ego velocity)
- distance deviation (invalid if the ego is too far from the trajectory)

## Inputs/Outputs

### Inputs

| Name                 | Type                                     | Description                                    |
| -------------------- | ---------------------------------------- | ---------------------------------------------- |
| `~/input/kinematics` | `nav_msgs/Odometry`                      | ego pose and twist                             |
| `~/input/trajectory` | `autoware_auto_planning_msgs/Trajectory` | target trajectory to be validated in this node |

### Outputs

| Name                         | Type                                         | Description                                                               |
| ---------------------------- | -------------------------------------------- | ------------------------------------------------------------------------- |
| `~/output/trajectory`        | `autoware_auto_planning_msgs/Trajectory`     | validated trajectory                                                      |
| `~/output/validation_status` | `planning_validator/PlanningValidatorStatus` | validator status to inform the reason why the trajectory is valid/invalid |
| `/diagnostics`               | `diagnostic_msgs/DiagnosticStatus`           | diagnostics to report errors                                              |

## Parameters

### System parameters

| Name                                 | Type   | Description                                                                                                              | Default value |
| :----------------------------------- | :----- | :----------------------------------------------------------------------------------------------------------------------- | :------------ |
| `publish_diag`                       | `bool` | if true, diagnostics msg is published.                                                                                   | true          |
| `use_previous_trajectory_on_invalid` | `bool` | if true, previous validated trajectory is published when the current trajectory is invalid. Otherwise none is published. | true          |
| `display_on_terminal`                | `bool` | show error msg on terminal                                                                                               | true          |

### Algorithm parameters

| Name         | Type     | Description | Default value |
| :----------- | :------- | :---------- | :------------ |
| `stop_decel` | `double` |             | 100.0         |
| `max_jerk`   | `double` |             | 2.0           |
| `min_jerk`   | `double` |             | 1.0           |
| `min_jerk`   | `double` |             | 9.8           |
| `min_jerk`   | `double` |             | 9.8           |
| `min_jerk`   | `double` |             | -9.8          |
| `min_jerk`   | `double` |             | 1.414         |
| `min_jerk`   | `double` |             | 10.0          |
| `min_jerk`   | `double` |             | 100.0         |
| `min_jerk`   | `double` |             | 100.0         |
