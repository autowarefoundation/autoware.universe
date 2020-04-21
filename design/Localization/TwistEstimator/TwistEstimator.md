Twist Estimator
==============

## Twist Estimator
Twist estimator is a component to estimate ego vehicle twist for precise velocity planning and control. The  x-axis velocity and z-axis angular velocity in vehicle twist is mainly considered. These values are preferable to be noise-free and unbiased.

## Input
| Input          | Data Type                                            |
|----------------|------------------------------------------------------|
| Vehicle CAN    | `geometry_msgs::TwistStamped`                        |
| IMU            | `sensor_msgs::Imu`                                   |

## Output

| Output            | Data Type                                   | Use Cases of the output         |
|-------------------|---------------------------------------------|---------------------------------|
| Estimated Twist   | `geometry_msgs::TwistWithCovarianceStamped` | Pose Twist Fusion Filter        |

## Design
 
![TwistEstimator](/design/img/TwistEstimator.svg)

In the figure above, the solid line of output shows our implementation, while the dotted lines of output show feasible implementations. Estimated twist compensates estimated pose in Pose Twist Fusion Filter, and also becomes the alternative to estimated pose when the calculation of Pose Estimator is highly unreliable. Therefore, estimated twist is preferable to be noise-free and unbiased. We adopt this design to merge multiple sensor inputs to generate more accurate twist.

