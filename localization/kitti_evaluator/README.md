# kitti_evaluator

## Purpose

`kitti_evaluator` is the package to evaluate localization output.

## Inputs / Outputs

### Input

| Name                            | Type                                             | Description                        |
| ------------------------------- | ------------------------------------------------ | ---------------------------------- |
| `vehicle/twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist with covariance from vehicle |
| `imu`                           | `sensor_msgs::msg::Imu`                          | imu from sensor                    |

### Output

| Name                    | Type                                             | Description                     |
| ----------------------- | ------------------------------------------------ | ------------------------------- |
| `twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | estimated twist with covariance |

## Parameters

| Parameter             | Type   | Description                      |
| --------------------- | ------ | -------------------------------- |
| `output_frame`        | String | output's frame id                |
| `message_timeout_sec` | Double | delay tolerance time for message |

## Assumptions / Known limits
