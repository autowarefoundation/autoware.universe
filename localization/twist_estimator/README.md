# twist_estimator

## Purpose

`twist_estimator` is a package to get the twist from sensor module and publish it to `ekf_localizer`.
The current implementation includes a function to set the velocity and angular velocity to 0 when the vehilce is stopped.

## Inputs / Outputs

### Input

| Name                                | Type                                             | Description                        |
| ----------------------------------- | ------------------------------------------------ | ---------------------------------- |
| `input_twist_with_covariance_topic` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist with covariance from sensors |

### Output

| Name                                 | Type                                             | Description                     |
| ------------------------------------ | ------------------------------------------------ | ------------------------------- |
| `output_twist_with_covariance_topic` | `geometry_msgs::msg::TwistWithCovarianceStamped` | estimated twist with covariance |

## Parameters

| Parameter             | Type   | Description                      |
| --------------------- | ------ | -------------------------------- |
| `message_timeout_sec` | Double | delay tolerance time for message |

## Assumptions / Known limits

- [Assumption] The angular velocity is set to zero if both the longitudinal vehicle velocity and the angular velocity around the yaw axis are sufficiently small. This is for suppression of the IMU angular velocity bias. Without this process, we misestimate the vehicle status when stationary.
