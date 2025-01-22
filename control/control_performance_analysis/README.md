# control_performance_analysis

## Purpose

`control_performance_analysis` is the package to analyze the tracking performance of a control module and monitor the driving status of the vehicle.

This package is used as a tool to quantify the results of the control module.
That's why it doesn't interfere with the core logic of autonomous driving.

Based on the various input from planning, control, and vehicle, it publishes the result of analysis as `control_performance_analysis::msg::ErrorStamped` defined in this package.

All results in `ErrorStamped` message are calculated in Frenet Frame of curve. Errors and velocity errors are calculated by using paper below.

<!-- cspell: ignore Werling Moritz Groell Lutz Bretthauer Georg -->

`Werling, Moritz & Groell, Lutz & Bretthauer, Georg. (2010). Invariant Trajectory Tracking With a Full-Size Autonomous Road Vehicle. IEEE Transactions on Robotics. 26. 758 - 765. 10.1109/TRO.2010.2052325.`

If you are interested in calculations, you can see the error and error velocity calculations in section `C. Asymptotical Trajectory Tracking With Orientation Control`.

Error acceleration calculations are made based on the velocity calculations above. You can see below the calculation of error acceleration.

![CodeCogsEqn](https://user-images.githubusercontent.com/45468306/169027099-ef15b306-2868-4084-a350-0e2b652c310f.png)

## Input / Output

### Input topics

| Name                                     | Type                                       | Description                                 |
| ---------------------------------------- | ------------------------------------------ | ------------------------------------------- |
| `/planning/scenario_planning/trajectory` | autoware_planning_msgs::msg::Trajectory    | Output trajectory from planning module.     |
| `/control/command/control_cmd`           | autoware_control_msgs::msg::Control        | Output control command from control module. |
| `/vehicle/status/steering_status`        | autoware_vehicle_msgs::msg::SteeringReport | Steering information from vehicle.          |
| `/localization/kinematic_state`          | nav_msgs::msg::Odometry                    | Use twist from odometry.                    |
| `/tf`                                    | tf2_msgs::msg::TFMessage                   | Extract ego pose from tf.                   |

### Output topics

| Name                                    | Type                                                     | Description                                         |
| --------------------------------------- | -------------------------------------------------------- | --------------------------------------------------- |
| `/control_performance/performance_vars` | control_performance_analysis::msg::ErrorStamped          | The result of the performance analysis.             |
| `/control_performance/driving_status`   | control_performance_analysis::msg::DrivingMonitorStamped | Driving status (acceleration, jerk etc.) monitoring |

### Outputs

#### control_performance_analysis::msg::DrivingMonitorStamped

| Name                         | Type  | Description                                                           |
| ---------------------------- | ----- | --------------------------------------------------------------------- |
| `longitudinal_acceleration`  | float | $[ \mathrm{m/s^2} ]$                                                  |
| `longitudinal_jerk`          | float | $[ \mathrm{m/s^3} ]$                                                  |
| `lateral_acceleration`       | float | $[ \mathrm{m/s^2} ]$                                                  |
| `lateral_jerk`               | float | $[ \mathrm{m/s^3} ]$                                                  |
| `desired_steering_angle`     | float | $[ \mathrm{rad} ]$                                                    |
| `controller_processing_time` | float | Timestamp between last two control command messages $[ \mathrm{ms} ]$ |

#### control_performance_analysis::msg::ErrorStamped

| Name                                       | Type  | Description                                                                                                                               |
| ------------------------------------------ | ----- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `lateral_error`                            | float | $[ \mathrm{m} ]$                                                                                                                          |
| `lateral_error_velocity`                   | float | $[ \mathrm{m/s} ]$                                                                                                                        |
| `lateral_error_acceleration`               | float | $[ \mathrm{m/s^2} ]$                                                                                                                      |
| `longitudinal_error`                       | float | $[ \mathrm{m} ]$                                                                                                                          |
| `longitudinal_error_velocity`              | float | $[ \mathrm{m/s} ]$                                                                                                                        |
| `longitudinal_error_acceleration`          | float | $[ \mathrm{m/s^2} ]$                                                                                                                      |
| `heading_error`                            | float | $[ \mathrm{rad} ]$                                                                                                                        |
| `heading_error_velocity`                   | float | $[ \mathrm{rad/s} ]$                                                                                                                      |
| `control_effort_energy`                    | float | $[ \mathbf{u}^\top \mathbf{R} \mathbf{u} ]$ simplified to $[ R \cdot u^2 ]$                                                               |
| `error_energy`                             | float | $e_{\text{lat}}^2 + e_\theta^2$ (squared lateral error + squared heading error)                                                           |
| `value_approximation`                      | float | $V = \mathbf{x}^\top \mathbf{P} \mathbf{x}$; Value function from DARE Lyapunov matrix $\mathbf{P}$                                        |
| `curvature_estimate`                       | float | $[ \mathrm{1/m} ]$                                                                                                                        |
| `curvature_estimate_pp`                    | float | $[ \mathrm{1/m} ]$                                                                                                                        |
| `vehicle_velocity_error`                   | float | $[ \mathrm{m/s} ]$                                                                                                                        |
| `tracking_curvature_discontinuity_ability` | float | Measures the ability to track curvature changes $\frac{\lvert \Delta(\text{curvature}) \rvert}{1 + \lvert \Delta(e_{\text{lat}}) \rvert}$ |

## Parameters

| Name                                  | Type             | Description                                                       |
| ------------------------------------- | ---------------- | ----------------------------------------------------------------- |
| `curvature_interval_length`           | double           | Used for estimating current curvature                             |
| `prevent_zero_division_value`         | double           | Value to avoid zero division. Default is `0.001`                  |
| `odom_interval`                       | unsigned integer | Interval between odom messages, increase it for smoother curve.   |
| `acceptable_max_distance_to_waypoint` | double           | Maximum distance between trajectory point and vehicle [m]         |
| `acceptable_max_yaw_difference_rad`   | double           | Maximum yaw difference between trajectory point and vehicle [rad] |
| `low_pass_filter_gain`                | double           | Low pass filter gain                                              |

## Usage

- After launched simulation and control module, launch the `control_performance_analysis.launch.xml`.
- You should be able to see the driving monitor and error variables in topics.
- If you want to visualize the results, you can use `Plotjuggler` and use `config/controller_monitor.xml` as layout.
- After import the layout, please specify the topics that are listed below.

> - /localization/kinematic_state
> - /vehicle/status/steering_status
> - /control_performance/driving_status
> - /control_performance/performance_vars

- In `Plotjuggler` you can export the statistic (max, min, average) values as csv file. Use that statistics to compare the control modules.

## Future Improvements

- Implement a LPF by cut-off frequency, differential equation and discrete state space update.
