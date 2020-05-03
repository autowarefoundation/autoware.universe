# Longitudinal Controller

# Overview

For following target trajectory, control stack needs to output lateral control commands (steering angle, steering angle velocty), and logitudinal control commands (acceleration, velocity). Logitudinal controller module is responsible for calculation of logitudinal control commands.

## Role

Longitudinal controller module calculates velocity and acceleration for following target velocity profile. This module velocity error from target trajectory.

### Input

The input to Longitudinal Controller module:
| Input | Data Type | Explanation |
| -----------| ------------------------------------ | ------------------------------------------------------------------ |
| Trajectory | `autoware_planning_msgs::Trajectory` | Target trajectory to follow (target position, twist, acceleration) |
| Pose | `/tf` <br>(`tf2_msgs::TFMessage`) | Current pose of the vehicle |
| Twist | `geometry_msgs::TwistStamped` | Current twist of the vehicle |

### Output

The output type from Longitudinal Controller module is `autoware_control_msgs/ControlCommandStamped`.

The main outputs included in `autoware_control_msgs/ControlCommandStamped` are as follows:

| Output                  | Data Type        |
| ----------------------- | ---------------- |
| Velocity                | std_msgs/Float64 |
| Acceleration            | std_msgs/Float64 |
| Steering angle          | std_msgs/Float64 |
| Steering angle velocity | std_msgs/Float64 |

Note that steering angle and steering angle velocity are always output as 0 from Longitudinal Controller module.
