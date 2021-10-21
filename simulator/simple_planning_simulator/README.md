# Simple Planning Simulator

This node simulates the vehicle motion for a vehicle command in 2D using a simple vehicle model.

## Interface

### input

- /control/vehicle_cmd [`autoware_vehicle_msgs/VehicleCommand`] : target command to drive a vehicle.
- /initialpose [`geometry_msgs/PoseWithCovarianceStamped`] : for initial pose
- /initialtwist [`geometry_msgs/TwistStamped`] : for initial velocity
- /planning/scenario_planning/trajectory [`autoware_planning_msgs/Trajectory`]: for z position
- /vehicle/engage : if true, the vehicle starts to move. if false, stops.

### output

- /tf [`tf2_msgs/TFMessage`] : simulated vehicle pose (base_link)
- /vehicle/status/control_mode [`autoware_vehicle_msgs/ControlMode`] : current control mode (Auto/Manual).
- /vehicle/status/shift [`autoware_vehicle_msgs/ShiftStamped`] : current shift (Fwd/Rev)
- /vehicle/status/steering [`autoware_vehicle_msgs/Steering`] : Simulated steering angle
- /vehicle/status/turn_signal [`autoware_vehicle_msgs/TurnSignal`] : current turn signal (just published with NONE status.)
- /vehicle/status/twist [`geometry_msgs/TwistStamped`] : simulated velocity

## Parameter Description

### Common Parameters

| Name                  | Type   | Description                                                    | Default value |
| :-------------------- | :----- | :------------------------------------------------------------- | :------------ |
| add_measurement_noise | bool   | If true, the Gaussian noise is added to the simulated results. | true          |
| pos_noise_stddev      | double | Standard deviation for position noise                          | 0.01          |
| rpy_noise_stddev      | double | Standard deviation for Euler angle noise                       | 0.0001        |
| vel_noise_stddev      | double | Standard deviation for longitudinal velocity noise             | 0.0           |
| angvel_noise_stddev   | double | Standard deviation for angular velocity noise                  | 0.0           |
| steer_noise_stddev    | double | Standard deviation for steering angle noise                    | 0.0001        |
| initial_engage_state  | double | If false, the engage command is needed to move the vehicle.    | true          |

### Vehicle Model Parameters

#### vehicle_model_type options

- `IDEAL_STEER`: uses velocity command. The steering and velocity varies ideally as commanded. The pose is calculated by a bicycle kinematics model.
- `IDEAL_ACCEL`: uses acceleration command. The steering and acceleration varies ideally as commanded. The pose is calculated by a bicycle kinematics model.
- `DELAY_STEER` : uses velocity command. The steering and velocity varies following a 1st-order delay model. The pose is calculated by a bicycle kinematics model.
- `DELAY_STEER_ACC` : uses acceleration command. The steering and acceleration varies following a 1st-order delay model. The pose is calculated by a bicycle kinematics model.

| Name                 | Type   | Description                                          | IDEAL STEER | IDEAL ACCEL | DELAY STEER | DELAY STEER 　 ACC | Default value | unit    |
| :------------------- | :----- | :--------------------------------------------------- | :---------- | :---------- | :---------- | :----------------- | :------------ | :------ |
| vel_time_delay       | double | dead time for the velocity input                     | x           | x           | o           | x                  | 0.25          | [s]     |
| acc_time_delay       | double | dead time for the acceleration input                 | x           | x           | x           | o                  | 0.1           | [s]     |
| steer_time_delay     | double | dead time for the steering input                     | x           | x           | o           | o                  | 0.24          | [s]     |
| vel_time_constant    | double | time constant of the 1st-order velocity dynamics     | x           | x           | o           | x                  | 0.61          | [s]     |
| acc_time_constant    | double | time constant of the 1st-order acceleration dynamics | x           | x           | x           | o                  | 0.1           | [s]     |
| steer_time_constant  | double | time constant of the 1st-order steering dynamics     | x           | x           | o           | o                  | 0.27          | [s]     |
| vel_lim              | double | limit of velocity                                    | x           | x           | o           | o                  | 50.0          | [m/s]   |
| accel_rate           | double | limit of acceleration                                | x           | x           | o           | o                  | 7.0           | [m/ss]  |
| steer_lim            | double | limit of steering angle                              | x           | x           | o           | o                  | 1.0           | [rad]   |
| steer_rate_lim       | double | limit of steering angle change rate                  | x           | x           | o           | o                  | 5.0           | [rad/s] |
| deadzone_delta_steer | double | dead zone for the steering dynamics                  | x           | x           | o           | o                  | 0.0           | [rad]   |

_Note_: The steering/velocity/acceleration dynamics is modeled by a first order system with a deadtime in a _delay_ model. The definition of the _time constant_ is the time it takes for the step response to rise up to 63% of its final value. The _deadtime_ is a delay in the response to a control input.
