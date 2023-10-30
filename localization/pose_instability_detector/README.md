# pose_instability_detector

The `pose_instability_detector` package includes a node designed to monitor the stability of `/localization/kinematic_state`, which is an output topic of the Extended Kalman Filter (EKF).

This node triggers periodic timer callbacks to compare two poses:

- The pose obtained by integrating the twist values from the last received message on `/localization/kinematic_state` over a duration specified by `interval_sec`.
- The latest pose from `/localization/kinematic_state`.

The results of this comparison are then output to the `/diagnostics` topic.

If this node outputs WARN messages to `/diagnostics`, it means that the EKF output is significantly different from the integrated twist values.
This discrepancy suggests that there may be an issue with either the `pose_estimator` or the `twist_estimator`.

The following diagram provides an overview of what the timeline of this process looks like:

![timeline](./media/timeline.drawio.svg)

## Parameters

See [pose_instability_detector.param.yaml](config/pose_instability_detector.param.yaml).

| Name                  | Type   | Description                         |
| --------------------- | ------ | ----------------------------------- |
| `interval_sec`        | double | Interval of timer callback (second) |
| `threshold_linear_x`  | double | Threshold of linear x (m)           |
| `threshold_linear_y`  | double | Threshold of linear y (m)           |
| `threshold_linear_z`  | double | Threshold of linear z (m)           |
| `threshold_angular_x` | double | Threshold of angular x (rad)        |
| `threshold_angular_y` | double | Threshold of angular y (rad)        |
| `threshold_angular_z` | double | Threshold of angular z (rad)        |

## Input

| Name               | Type                                           | Description           |
| ------------------ | ---------------------------------------------- | --------------------- |
| `~/input/odometry` | nav_msgs::msg::Odometry                        | Pose estimated by EKF |
| `~/input/twist`    | geometry_msgs::msg::TwistWithCovarianceStamped | Twist                 |

## Output

| Name           | Type                                  | Description |
| -------------- | ------------------------------------- | ----------- |
| `/diagnostics` | diagnostic_msgs::msg::DiagnosticArray | Diagnostics |

![rqt_runtime_monitor](./media/rqt_runtime_monitor.png)
