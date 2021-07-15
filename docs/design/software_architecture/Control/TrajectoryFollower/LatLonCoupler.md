# Latlon Coupler

## Overview

## Role

Latlon Coupler module integrates lateral control command and longitudinal control command. For integration, newest value of each command are used as vehicle control command.

### Input

#### from Lateral Control module

`autoware_control_msgs/ControlCommandStamped`:

| Input                   | Data Type        | Explanation |
| ----------------------- | ---------------- | ----------- |
| Velocity                | std_msgs/Float64 | (Zero)      |
| Acceleration            | std_msgs/Float64 | (Zero)      |
| Steering angle          | std_msgs/Float64 |             |
| Steering angle velocity | std_msgs/Float64 |             |

#### from Longitudinal Control module

`autoware_control_msgs/ControlCommandStamped`:

| Input                   | Data Type        | Explanation |
| ----------------------- | ---------------- | ----------- |
| Velocity                | std_msgs/Float64 |             |
| Acceleration            | std_msgs/Float64 |             |
| Steering angle          | std_msgs/Float64 | (Zero)      |
| Steering angle velocity | std_msgs/Float64 | (Zero)      |

### Output

`autoware_control_msgs/ControlCommandStamped`:

| Input                   | Data Type        | Explanation                      |
| ----------------------- | ---------------- | -------------------------------- |
| Velocity                | std_msgs/Float64 | from Longitudinal Control module |
| Acceleration            | std_msgs/Float64 | from Longitudinal Control module |
| Steering angle          | std_msgs/Float64 | from Lateral Control module      |
| Steering angle velocity | std_msgs/Float64 | from Lateral Control module      |
