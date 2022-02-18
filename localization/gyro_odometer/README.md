# gyro_odometer

## Purpose

`gyro_odometer` is the package to estimate twist by combining imu and vehicle speed.

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

| Parameter      | Type   | Description       |
| -------------- | ------ | ----------------- |
| `output_frame` | String | output's frame id |

## Assumptions / Known limits

- [Assumption] The frame_id of input twist message must be base_link

- [Assumption] The covariance in the input messages must be properly assigned

- [Assumption] If both velocity and angular velocity are sufficiently small, the angular velocity is fixed at zero
- This is because there was a bias in the angular velocity of the IMU, which sometimes caused a estimation that the vehicle was rotating even though it is actually stopped.

- [Limitation] The frequency of the output IMU message depends on the frequency of the input IMU message

- [Limitation] The variances of linear.y and linear.z are output message is assigned large values
