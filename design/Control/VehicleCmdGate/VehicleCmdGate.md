# Vehicle Cmd Gate

# Overview

Vehicle Cmd Gate module is responsible for Systematic post-processing.

## Role

Roles of Vehicle Cmd Gate module are as follows.

- Reshape the vehicle control command
  - Vehicle Cmd Gate module convert `autoware_control_msgs/ControlCommandStamped` to `autoware_vehicle_msgs/VehicleCommand`. This conversion includes the addtion of gear shifting command.
- Reflect engage command to control signal for vehicles
  - Until true command is sent as engage command, Vehicle Cmd Gate module does not pass the input command information as output.
- Select the command values (Trajectory follow command, Remote manual command)
- Observe the maximum speed limit, maximum lateral/longitudinal jerk
- Stop urgently when emergency command is received

### Input

- Control commands from Trajectory Follower module(`autoware_control_msgs/ControlCommandStamped`)
- Remote Control commands(Undefined[TBD])
- Engage Commands(std_msgs/Bool)

The main inputs included in `autoware_control_msgs/ControlCommandStamped` are as follows:

| Input                   | Data Type        |
| ----------------------- | ---------------- |
| Velocity                | std_msgs/Float64 |
| Acceleration            | std_msgs/Float64 |
| Steering angle          | std_msgs/Float64 |
| Steering angle velocity | std_msgs/Float64 |

### Output

- Control signal for vehicles (autoware_vehicle_msgs/VehicleCommand)

The main inputs included in `autoware_vehicle_msgs/VehicleCommand` are as follows:

| Output                  | Data Type        |
| ----------------------- | ---------------- |
| Velocity                | std_msgs/Float64 |
| Acceleration            | std_msgs/Float64 |
| Steering angle          | std_msgs/Float64 |
| Steering angle velocity | std_msgs/Float64 |
| Gear shifting command   | std_msgs/Int32   |
| Emergency command       | std_msgs/Int32   |

Note that most of above commands are ouput as 0 until true command is sent as engage command.
