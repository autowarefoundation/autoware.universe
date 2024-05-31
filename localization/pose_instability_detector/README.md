# pose_instability_detector

The `pose_instability_detector` package includes a node designed to monitor the stability of `/localization/kinematic_state`, which is an output topic of the Extended Kalman Filter (EKF).

This node triggers periodic timer callbacks to compare two poses:

- The pose calculated by dead reckoning starting from the pose of `/localization/kinematic_state` obtained `timer_period` seconds ago.
- The latest pose from `/localization/kinematic_state`.

The results of this comparison are then output to the `/diagnostics` topic.

If this node outputs WARN messages to `/diagnostics`, it means that the EKF output is significantly different from the integrated twist values.
This discrepancy suggests that there may be an issue with either the estimated pose or the input twist.

The following diagram provides an overview of how the procedure looks like:

![procedure_overview](./media/pose_instabilty_detector_overview.svg)

## Dead reckoning algorithm

Dead reckoning is a method of estimating the position of a vehicle based on its previous position and velocity.
The procedure for dead reckoning is as follows:

1. Capture the necessary twist values from the `/input/twist` topic.
2. Integrate the twist values to calculate the pose transition.
3. Apply the pose transition to the previous pose to obtain the current pose.

### Collecting twist values

The `pose_instability_detector` node collects the twist values from the `~/input/twist` topic to perform dead reckoning.
Ideally, `pose_instability_detector` needs the twist values between the previous pose and the current pose.
Threfore, `pose_instability_detector` snips the twist buffer and apply interpolations and extrapolations to obtain the twist values at the desired time.

![how_to_snip_necessary_twist](./media/how_to_snip_twist.png)

### Linear transition

### Angular transition

## Threshold definition

## Parameters

{{ json_to_markdown("localization/pose_instability_detector/schema/pose_instability_detector.schema.json") }}

## Input

| Name               | Type                                           | Description           |
| ------------------ | ---------------------------------------------- | --------------------- |
| `~/input/odometry` | nav_msgs::msg::Odometry                        | Pose estimated by EKF |
| `~/input/twist`    | geometry_msgs::msg::TwistWithCovarianceStamped | Twist                 |

## Output

| Name                | Type                                  | Description |
| ------------------- | ------------------------------------- | ----------- |
| `~/debug/diff_pose` | geometry_msgs::msg::PoseStamped       | diff_pose   |
| `/diagnostics`      | diagnostic_msgs::msg::DiagnosticArray | Diagnostics |

![rqt_runtime_monitor](./media/rqt_runtime_monitor.png)
