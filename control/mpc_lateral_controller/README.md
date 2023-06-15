# MPC Lateral Controller

This is the design document for the lateral controller node
in the `trajectory_follower_node` package.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This node is used to general lateral control commands (steering angle and steering rate)
when following a path.

## Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The node uses an implementation of linear model predictive control (MPC) for accurate path tracking.
The MPC uses a model of the vehicle to simulate the trajectory resulting from the control command.
The optimization of the control command is formulated as a Quadratic Program (QP).

Different vehicle models are implemented:

- kinematics : bicycle kinematics model with steering 1st-order delay.
- kinematics_no_delay : bicycle kinematics model without steering delay.
- dynamics : bicycle dynamics model considering slip angle.
  The kinematics model is being used by default. Please see the reference [1] for more details.

For the optimization, a Quadratic Programming (QP) solver is used and two options are currently implemented:

<!-- cspell: ignore ADMM -->

- unconstraint_fast : use least square method to solve unconstraint QP with eigen.
- [osqp](https://osqp.org/): run the [following ADMM](https://web.stanford.edu/~boyd/papers/admm_distr_stats.html)
  algorithm (for more details see the related papers at
  the [Citing OSQP](https://web.stanford.edu/~boyd/papers/admm_distr_stats.html) section):

### Filtering

Filtering is required for good noise reduction.
A [Butterworth filter](https://en.wikipedia.org/wiki/Butterworth_filter) is used for the yaw and lateral errors used as
input of the MPC as well as for
the output steering angle.
Other filtering methods can be considered as long as the noise reduction performances are good
enough.
The moving average filter for example is not suited and can yield worse results than without any
filtering.

## Assumptions / Known limits

<!-- Required -->

The tracking is not accurate if the first point of the reference trajectory is at or in front of the current ego pose.

## Inputs / Outputs / API

<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Inputs

Set the following from the [controller_node](../trajectory_follower_node/README.md)

- `autoware_auto_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current odometry
- `autoware_auto_vehicle_msgs/SteeringReport` current steering

### Outputs

Return LateralOutput which contains the following to the controller node

- `autoware_auto_control_msgs/AckermannLateralCommand`
- LateralSyncData
  - steer angle convergence

### MPC class

The `MPC` class (defined in `mpc.hpp`) provides the interface with the MPC algorithm.
Once a vehicle model, a QP solver, and the reference trajectory to follow have been set
(using `setVehicleModel()`, `setQPSolver()`, `setReferenceTrajectory()`), a lateral control command
can be calculated by providing the current steer, velocity, and pose to function `calculateMPC()`.

### Parameter description

The default parameters defined in `param/lateral_controller_defaults.param.yaml` are adjusted to the
AutonomouStuff Lexus RX 450h for under 40 km/h driving.

| Name                                         | Type   | Description                                                                                                                                       | Default value |
| :------------------------------------------- | :----- | :------------------------------------------------------------------------------------------------------------------------------------------------ | :------------ |
| show_debug_info                              | bool   | display debug info                                                                                                                                | false         |
| traj_resample_dist                           | double | distance of waypoints in resampling [m]                                                                                                           | 0.1           |
| enable_path_smoothing                        | bool   | path smoothing flag. This should be true when uses path resampling to reduce resampling noise.                                                    | true          |
| path_filter_moving_ave_num                   | int    | number of data points moving average filter for path smoothing                                                                                    | 35            |
| path_smoothing_times                         | int    | number of times of applying path smoothing filter                                                                                                 | 1             |
| curvature_smoothing_num_ref_steer            | double | index distance of points used in curvature calculation for reference steer command: p(i-num), p(i), p(i+num). larger num makes less noisy values. | 35            |
| curvature_smoothing_num_traj                 | double | index distance of points used in curvature calculation for trajectory: p(i-num), p(i), p(i+num). larger num makes less noisy values.              | 1             |
| extend_trajectory_for_end_yaw_control        | bool   | trajectory extending flag for end yaw control.                                                                                                    | true          |
| steering_lpf_cutoff_hz                       | double | cutoff frequency of lowpass filter for steering output command [hz]                                                                               | 3.0           |
| admissible_position_error                    | double | stop vehicle when following position error is larger than this value [m].                                                                         | 5.0           |
| admissible_yaw_error_rad                     | double | stop vehicle when following yaw angle error is larger than this value [rad].                                                                      | 1.57          |
| stop_state_entry_ego_speed <sup>\*1</sup>    | double | threshold value of the ego vehicle speed used to the stop state entry condition                                                                   | 0.0           |
| stop_state_entry_target_speed <sup>\*1</sup> | double | threshold value of the target speed used to the stop state entry condition                                                                        | 0.0           |
| converged_steer_rad                          | double | threshold value of the steer convergence                                                                                                          | 0.1           |
| keep_steer_control_until_converged           | bool   | keep steer control until steer is converged                                                                                                       | true          |
| new_traj_duration_time                       | double | threshold value of the time to be considered as new trajectory                                                                                    | 1.0           |
| new_traj_end_dist                            | double | threshold value of the distance between trajectory ends to be considered as new trajectory                                                        | 0.3           |
| mpc_converged_threshold_rps                  | double | threshold value to be sure output of the optimization is converged, it is used in stopped state                                                   | 0.3           |

(\*1) To prevent unnecessary steering movement, the steering command is fixed to the previous value in the stop state.

#### MPC algorithm

| Name                                                | Type   | Description                                                                                                                                        | Default value     |
| :-------------------------------------------------- | :----- | :------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------- |
| qp_solver_type                                      | string | QP solver option. described below in detail.                                                                                                       | unconstraint_fast |
| vehicle_model_type                                  | string | vehicle model option. described below in detail.                                                                                                   | kinematics        |
| prediction_horizon                                  | int    | total prediction step for MPC                                                                                                                      | 70                |
| prediction_sampling_time                            | double | prediction period for one step [s]                                                                                                                 | 0.1               |
| weight_lat_error                                    | double | weight for lateral error                                                                                                                           | 0.1               |
| weight_heading_error                                | double | weight for heading error                                                                                                                           | 0.0               |
| weight_heading_error_squared_vel_coeff              | double | weight for heading error \* velocity                                                                                                               | 5.0               |
| weight_steering_input                               | double | weight for steering error (steer command - reference steer)                                                                                        | 1.0               |
| weight_steering_input_squared_vel_coeff             | double | weight for steering error (steer command - reference steer) \* velocity                                                                            | 0.25              |
| weight_lat_jerk                                     | double | weight for lateral jerk (steer(i) - steer(i-1)) \* velocity                                                                                        | 0.0               |
| weight_terminal_lat_error                           | double | terminal cost weight for lateral error                                                                                                             | 1.0               |
| weight_steer_rate                                   | double | weight for steering rate [rad/s]                                                                                                                   | 0.0               |
| weight_steer_acc                                    | double | weight for derivatives of the steering rate [rad/ss]                                                                                               | 0.0               |
| weight_terminal_heading_error                       | double | terminal cost weight for heading error                                                                                                             | 0.1               |
| zero_ff_steer_deg                                   | double | threshold of feedforward angle [deg]. feedforward angle smaller than this value is set to zero.                                                    | 2.0               |
| mpc_low_curvature_thresh_curvature                  | double | trajectory curvature threshold to change the weight. If the curvature is lower than this value, the `low_curvature_weight_**` values will be used. | 0.0               |
| mpc_low_curvature_weight_lat_error                  | double | [used in a low curvature trajectory] weight for lateral error                                                                                      | 0.0               |
| mpc_low_curvature_weight_heading_error              | double | [used in a low curvature trajectory] weight for heading error                                                                                      | 0.0               |
| mpc_low_curvature_weight_heading_error_squared_vel  | double | [used in a low curvature trajectory] weight for heading error \* velocity                                                                          | 0.0               |
| mpc_low_curvature_weight_steering_input             | double | [used in a low curvature trajectory] weight for steering error (steer command - reference steer)                                                   | 0.0               |
| mpc_low_curvature_weight_steering_input_squared_vel | double | [used in a low curvature trajectory] weight for steering error (steer command - reference steer) \* velocity                                       | 0.0               |
| mpc_low_curvature_weight_lat_jerk                   | double | [used in a low curvature trajectory] weight for lateral jerk (steer(i) - steer(i-1)) \* velocity                                                   | 0.0               |
| mpc_low_curvature_weight_steer_rate                 | double | [used in a low curvature trajectory] weight for steering rate [rad/s]                                                                              | 0.0               |
| mpc_low_curvature_weight_steer_acc                  | double | [used in a low curvature trajectory] weight for derivatives of the steering rate [rad/ss]                                                          | 0.0               |

#### Vehicle

| Name          | Type   | Description                                                              | Default value |
| :------------ | :----- | :----------------------------------------------------------------------- | :------------ |
| cg_to_front_m | double | distance from baselink to the front axle[m]                              | 1.228         |
| cg_to_rear_m  | double | distance from baselink to the rear axle [m]                              | 1.5618        |
| mass_fl       | double | mass applied to front left tire [kg]                                     | 600           |
| mass_fr       | double | mass applied to front right tire [kg]                                    | 600           |
| mass_rl       | double | mass applied to rear left tire [kg]                                      | 600           |
| mass_rr       | double | mass applied to rear right tire [kg]                                     | 600           |
| cf            | double | front cornering power [N/rad]                                            | 155494.663    |
| cr            | double | rear cornering power [N/rad]                                             | 155494.663    |
| steering_tau  | double | steering dynamics time constant (1d approximation) for vehicle model [s] | 0.3           |

### How to tune MPC parameters

#### Set kinematics information

First, it's important to set the appropriate parameters for vehicle kinematics. This includes parameters like `wheelbase`, which represents the distance between the front and rear wheels, and `max_steering_angle`, which indicates the maximum tire steering angle. These parameters should be set in the `vehicle_info.param.yaml`.

#### Set dynamics information

Next, you need to set the proper parameters for the dynamics model. These include the time constant `steering_tau` for steering dynamics, and the maximum acceleration `mpc_acceleration_limit` and the time constant `mpc_velocity_time_constant` for velocity dynamics.

#### Confirmation of the input information

It's also important to make sure the input information is accurate. Information such as the velocity of the center of the rear wheel [m/s] and the steering angle of the tire [rad] is required. Please note that there have been frequent reports of performance degradation due to errors in input information. For instance, there are cases where the velocity of the vehicle is offset due to an unexpected difference in tire radius, or the tire angle cannot be accurately measured due to a deviation in the steering gear ratio or midpoint. It is suggested to compare information from multiple sensors (e.g., integrated vehicle speed and GNSS position, steering angle and IMU angular velocity), and ensure the input information for MPC is appropriate.

#### MPC weight tuning

Then, tune the weights of the MPC. The basic steps are as follows:

1. Set `weight_steering_input` = 1.0, `weight_lat_error` = 0.1, and other weights to 0.
2. If the vehicle oscillates when driving at low speed, set `weight_steering_input` larger.
3. If the vehicle oscillates when driving at high speed, set `mpc_weight_steering_input_squared_vel` larger.

#### Other tips

Here are some tips for adjusting other parameters:

- One of the simple way for tuning is to increase `weight_lat_error` until oscillation occurs.
- If the vehicle is unstable with very small `weight_lat_error`, increase terminal weight :`weight_terminal_lat_error` and `weight_terminal_heading_error` to improve tracking stability.
- Larger `prediction_horizon` and smaller `prediction_sampling_time` is effective for tracking performance, but it is a trade-off between computational costs.
- When you want to change the weight based on the trajectory curvature (e.g., large weight when driving on a sharp curve), use `mpc_low_curvature_thresh_curvature` and set `mpc_low_curvature_weight_**` weights.
- `weight_lat_error`: Reduce lateral tracking error. This acts like P gain in PID.
- `weight_heading_error`: Make a drive straight. This acts like D gain in PID.
- `weight_heading_error_squared_vel_coeff` : Make a drive straight in high speed range.
- `weight_steering_input`: Reduce oscillation of tracking.
- `weight_steering_input_squared_vel_coeff`: Reduce oscillation of tracking in high speed range.
- `weight_lat_jerk`: Reduce lateral jerk.
- `weight_terminal_lat_error`: Preferable to set a higher value than normal lateral weight `weight_lat_error` for
  stability.
- `weight_terminal_heading_error`: Preferable to set a higher value than normal heading weight `weight_heading_error`
  for stability.

## References / External links

<!-- Optional -->

- [1] Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking",
  Robotics Institute, Carnegie Mellon University, February 2009.

## Related issues

<!-- Required -->
