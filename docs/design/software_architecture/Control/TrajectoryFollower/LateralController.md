# Lateral Controller

# Overview

For following target trajectory, control stack needs to output lateral control commands (steering angle, steering angle velocity), and longitudinal control commands (acceleration, velocity). Lateral controller module is responsible for calculation of lateral control commands.

## Role

Lateral controller module calculates suitable velocity and acceleration for following target trajectory.

### Input

The input to Lateral Controller module:

| Input      | Data Type                            | Explanation                                                                     |
| ---------- | ------------------------------------ | ------------------------------------------------------------------------------- |
| Trajectory | `autoware_planning_msgs::Trajectory` | Target trajectory to follow (target position, orientation, twist, acceleration) |
| Pose       | `/tf` <br>(`tf2_msgs::TFMessage`)    | Current pose of the vehicle                                                     |
| Twist      | `geometry_msgs::TwistStamped`        | Current twist of the vehicle                                                    |
| Steer      | `autoware_vehicle_msgs::Steering`    | Current steer of the vehicle                                                    |

### Output

The output type from Lateral Controller module is `autoware_control_msgs/ControlCommandStamped`.

The main outputs included in `autoware_control_msgs/ControlCommandStamped` are as follows:

| Output                  | Data Type        |
| ----------------------- | ---------------- |
| Velocity                | std_msgs/Float64 |
| Acceleration            | std_msgs/Float64 |
| Steering angle          | std_msgs/Float64 |
| Steering angle velocity | std_msgs/Float64 |

Note that velocity and acceleration are always output as 0 from Lateral Controller module.
