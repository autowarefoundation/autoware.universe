# twist_estimator

## Purpose

`twist_estimator` is the package to estimate twist by combining imu and vehicle speed.

## Inputs / Outputs

### Input

| Name                            | Type                                             | Description                        |
| ------------------------------- | ------------------------------------------------ | ---------------------------------- |
| `sensing_twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist with covariance from sensors |

### Output

| Name                                 | Type                                             | Description                     |
| ------------------------------------ | ------------------------------------------------ | ------------------------------- |
| `localization_twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | estimated twist with covariance |

## Parameters

| Parameter             | Type   | Description                      |
| --------------------- | ------ | -------------------------------- |
| `message_timeout_sec` | Double | delay tolerance time for message |

## Assumptions / Known limits

- [Assumption] The angular velocity is set to zero if both the longitudinal vehicle velocity and the angular velocity around the yaw axis are sufficiently small. This is for suppression of the IMU angular velocity bias. Without this process, we misestimate the vehicle status when stationary.
