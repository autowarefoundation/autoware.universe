# ssc_interface

`ssc_interface` is the package to connect Autoware with the AutonomouStuff Speed and Steering Control (SSC)*.

* SSC ... https://autonomoustuff.com/products/astuff-speed-steering-control-software

## Input / Output

### Input topics

- From Autoware

  | Name                       | Type                                       | Description                              |
  | -------------------------- | ------------------------------------------ | ---------------------------------------- |
  | `/control/vehicle_cmd`     | autoware_vehicle_msgs::msg::VehicleCommand | lateral and longitudinal control command |
  | `/control/turn_signal_cmd` | autoware_vehicle_msgs::msg::TurnSignal     | turn indicators command                  |
  | `/vehicle/engage`          | autoware_vehicle_msgs::msg::Engage         | engage command                           |

- From SSC

  | Name                               | Type                                             | Description                       |
  | ---------------------------------- | ------------------------------------------------ | --------------------------------- |
  | `as/velocity_accel_cov`            | automotive_platform_msgs::msg::VelocityAccelCov  | current velocity and acceleration |
  | `as/curvature_feedback`            | automotive_platform_msgs::msg::CurvatureFeedback | current curvature                 |
  | `as/throttle_feedback`             | automotive_platform_msgs::msg::ThrottleFeedback  | current accel pedal               |
  | `as/brake_feedback`                | automotive_platform_msgs::msg::BrakeFeedback     | current brake pedal               |
  | `as/gear_feedback`                 | automotive_platform_msgs::msg::GearFeedback      | current gear status               |
  | `pacmod/parsed_tx/wheel_speed_rpt` | pacmod3_msgs::msg::WheelSpeedRpt                 | current steering wheel speed      |
  | `pacmod/parsed_tx/steer_rpt`       | pacmod3_msgs::msg::SystemRptFloat                | current steering wheel angle      |

### Output topics

- To SSC

  | Name                              | Type                                             | Description                  |
  | --------------------------------- | ------------------------------------------------ | ---------------------------- |
  | `as/arbitrated_speed_commands`    | automotive_platform_msgs::msg::SpeedMode         | speed command                |
  | `as/arbitrated_steering_commands` | automotive_platform_msgs::msg::SteerMode         | steering wheel angle command |
  | `as/turn_signal_command`          | automotive_platform_msgs::msg::TurnSignalCommand | turn indicators command      |

- To Autoware

  | Name                                     | Type                                     | Description            |
  | ---------------------------------------- | ---------------------------------------- | ---------------------- |
  | `/vehicle/status/control_mode`           | autoware_vehicle_msgs::msg::ControlMode  | control mode           |
  | `/vehicle/status/velocity_status`        | geometry_msgs::msg::TwistStamped         | velocity               |
  | `/vehicle/status/steering_status`        | automotive_platform_msgs::msg::SteerMode | steering wheel angle   |
  | `/vehicle/status/turn_signal`            | autoware_vehicle_msgs::msg::ShiftStamped | gear status            |
  | `/vehicle/status/turn_indicators_status` | autoware_vehicle_msgs::msg::TurnSignal   | turn indicators status |

## ROS Parameters

 | Name                      | Type   | Description                                                                           |
 | ------------------------- | ------ | ------------------------------------------------------------------------------------- |
 | `use_rear_wheel_speed`    | bool   | use rear wheel speed to calculate vehicle speed or not                                |
 | `use_adaptive_gear_ratio` | bool   | use adaptive gear ratio to calculate current steering wheel angle from steering angle |
 | `ssc_gear_ratio_`         | double | default gear ratio ( it used only when `use_adaptive_gear_ratio` is False )           |
 | `command_timeout`         | double | timeout [ms]                                                                          |
 | `loop_rate`               | double | loop rate to publish *commands*                                                       |
 | `acceleration_limit`      | double | maximum accleration sent to SSC                                                       |
 | `deceleration_limit`      | double | maximum deceleration (absolute value) sent to SSC                                     |
 | `max_curvature_rate`      | double | maximum curvature rate   sent to SSC                                                  |
 | `agr_coef_a`              | double | coefficient to calculate adaptive gear ratio for steering wheel angle                 |
 | `agr_coef_b`              | double | coefficient to calculate adaptive gear ratio for steering wheel angle                 |
 | `agr_coef_c`              | double | coefficient to calculate adaptive gear ratio for steering wheel angle                 |
 | `steering_offset`         | double | steering wheel angle offset                                                           |