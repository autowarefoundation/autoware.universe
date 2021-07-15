# Control

## Overview

Control stack generates control signals to drive a vehicle following trajectories considering vehicle dynamics.
This layer ensures that the vehicle follows the trajectory planned by planning.
The output of Control stack includes velocity, acceleration, and steering.

## Role

There are two main roles of Control Stack:

- **Generation of control command for following target trajectory**
- **Post-processing of vehicle control command**

## Use Case

Control stack supports the following use cases.

1. Driving without excessive speed
2. Driving at slope
3. Smooth stop by normal obstacles / Sudden stop by obstacle's running-out
4. Forward/Reverse parking

## Requirement

To achieve the above use case, Control stack requires the following conditions.

- The input trajectory includes speed limit at each point (Use case 1).
- The input pose includes gradient information (=vehicle orientation) (Use case 2).
- The output vehicle command includes acceleration but also velocity (Use case 2, 3).
- The output vehicle command includes the command to shift drive/reverse gear(Use case 4.).

## Input

The input to Control stack:

| Input          | Topic (Data Type)                                                                   | Explanation                             |
| -------------- | ----------------------------------------------------------------------------------- | --------------------------------------- |
| Trajectory     | `/planning/scenario_planning/trajectory` <br>(`autoware_planning_msgs::Trajectory`) | Target trajectory to follow             |
| Pose           | `/tf` <br>(`tf2_msgs::TFMessage`)                                                   | Current pose of the vehicle             |
| Twist          | `/vehicle/status/twist` <br> (`geometry_msgs::TwistStamped`)                        | Current twist of the vehicle            |
| Steer          | `/vehicle/status/steering`<br>(`autoware_vehicle_msgs::Steering`)                   | Current steer of the vehicle            |
| Engage Command | `/autoware/engage`<br>(`std_msgs::Bool`)                                            | Whether to send commands to the vehicle |
| Remote Command | -                                                                                   | Control command from remote             |

As the above requirements, the data type of target trajectory, `autoware_planning_msgs::Trajectory`, includes the speed at each point.

### Output

The table below summarizes the output from Control stack:

| Output          | Topic(Data Type)                                                   | Explanation |
| --------------- | ------------------------------------------------------------------ | ----------- |
| Vehicle Command | `/control/vehicle_cmd`<br>(`autoware_vehicle_msgs/VehicleCommand`) | Table Below |

The main outputs included in Vehicle Command are as follows.

| Output                  | Data Type        |
| ----------------------- | ---------------- |
| Velocity                | std_msgs/Float64 |
| Acceleration            | std_msgs/Float64 |
| Steering angle          | std_msgs/Float64 |
| Steering angle velocity | std_msgs/Float64 |
| Gear shifting command   | std_msgs/Int32   |
| Emergency command       | std_msgs/Int32   |

As above requirements, the control stack outputs gear shifting command and acceleration command as Vehicle command

## Design

![ControlOverview](image/ControlOverview.svg)

## Trajectory follower

### Role

Generate control command for following given trajectory smoothly.

### Input

- Target trajectory
  - Target trajectory includes target position, target twist, target acceleration
- Current pose
- Current velocity
- Current steering

### Output

- Steering angle command
- Steering angular velocity command
- Velocity command
- Acceleration command

## Vehicle command gate

### Role

Systematic post-processing of vehicle control command, independent of trajectory following process

- Reshape the vehicle control command
- Select the command values (Trajectory follow command, Remote manual command)
- Observe the maximum speed limit, maximum lateral/longitudinal jerk
- Stop urgently when an emergency command is received

### Input

- Control commands from Trajectory Follower module
- Remote Control commands
- Engage Commands

### Output

- Control signal for vehicles

## References

TBU
