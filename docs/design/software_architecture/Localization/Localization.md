Localization
=============

# Overview

The localization stack has a role to recognize where ego vehicle is ["map" frame](/design/Tf.md). Additionally, this stack estimates twist of ego vehicle for precise velocity planning and control.

## Role

There are two main roles of Localization stack:

- Estimation of a self-position and a self-twist
- Integration of pose and twist information estimated by multiple sensors for robustness

## Input

| Input          | Data Type                                  |
| -------------- | ------------------------------------------ |
| LiDAR          | `sensor_msgs::PointCoud2`                  |
| GNSS           | `geometry_msgs::PoseWithCovarianceStamped` |
| IMU            | `sensor_msgs::Imu`                         |
| Pointcloud Map | `sensor_msgs::PointCoud2`                  |
| Vehicle CAN    | `geometry_msgs::TwistStamped`              |

### Sensors

Multiple sensor information described below is considered.

- LiDAR

  Pointcloud registration method such as ICP, NDT estimates ego vehicle pose by refining the relative transformation between 3D point cloud from lidar with reference pointcloud map.

- GNSS

  The pose information received from GNSS is projected into "map" frame. This can be used as one of the pose estimator mentioned below or it can be used to provide an initial guess for a sequential localization method.

- IMU

  Angular velocity from IMU is used as the vehicle twist.

- Vehicle CAN

  Vehicle CAN outputs useful information such as vehicle velocity, steering wheel angle to estimate vehicle twist. We adapt vehicle velocity from vehicle CAN as vehicle twist.

- Camera

  We do not implement camera-based pose or twist estimator. You can easily integrate image-based-estimator such as visual odometry and visual slam into the localization stack.

### Reference Map

- Pointcloud Map
  
## Output

| Output        | Topic (Data Type)                                       | Use Cases of the output       |
| ------------- | ------------------------------------------------------- | ----------------------------- |
| Vehicle Pose  | `/tf` <br> (`tf2_msgs/TFMessage`)                       | Perception, Planning, Control |
| Vehicle Twist | `/localization/twist`<br>(`geometry_msgs/TwistStamped`) | Planning, Control             |

## Use Cases

| Use Cases                                                | Requirement in `Localization`      | Output                 | How it is used                                                                                                                                                     |
| -------------------------------------------------------- | ---------------------------------- | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 1. Passing intersection<br>with traffic lights           | Self pose on the map               | Perception             | To detect traffic lights associated with the lane<br>where ego vehicle is in the camera image                                                                      |
| 2. Changing lane                                         | Self pose on the map               | Perception<br>Planning | To predict object motion on the lane<br>with lane information<br><br>To recognize drivable area based on lane information<br>and the position where ego vehicle is |
| 3. Stopping at crosswalk<br>when a pedestrian is walking | Self pose on the map               | Perception             | To recognize where the crosswalk is<br>based on ego vehicle position and map information                                                                           |
| 4. Reaching a goal<br>by driving on lanes                | Self pose on the map               | Planning               | To plan the global path from the position where ego vehicle is to<br>a goal with lane information                                                                  |
| 5. Driving<br>with following speed limits                | Self pose on the map<br>Self twist | Planning               | To recognize speed limit on the lane<br>where ego vehicle is<br><br>To plan target velocity<br>based on the velocity of ego vehicle and speed limit                |
| 6. Driving<br>on the target lane                         | Self pose on the map<br>Self twist | Control                | To calculate target throttle/brake value and steering angle<br>based on pose and twist of ego vehicle and target trajectory                                        |

## Requirements
The high-level requirements of Localization stack are listed below:
* Localization stack must provide pose in "map" frame. (Use Case 1-6)
  * The output should be provided as TF from "map" to "base_link". (See [TF document](/design/TF.md) for the details)
  * The localization result must be continuous 
  * Whenever a localization algorithm fails, the failure must be detected should not update vehicle pose.
* Localization stack must provide the velocity of the vehicle in "map" frame. (Use Case 5,6)

# Design

The localization stack provides indispensable information to achieve autonomous driving. Therefore, it is not preferable to depend on only one localization algorithm. We insert pose twist fusion filter after pose and twist estimator to improve robustness of the estimated pose and twist. Also, developers can easily add a new estimator based on another sensor, e.g. camera-based visual SLAM and visual odometry, into the localization stack.  The localization stack should output the transformation from map to base_link as /tf to utilize its interpolation system. 

![Localization_component](image/LocalizationOverview.svg)

## Pose estimator

### Role
Pose estimator is a component to estimate ego vehicle pose which includes position and orientation. The final output should also include covariance, which represents the estimator's confidence on the estimated pose. A pose estimator could either estimate pose in a local map, or it can estimate absolute pose using global localizer. The output pose can be published in any frame as long as enough /tf is provided to project into the "map" frame.

### Input

- LiDAR
- GNSS
- Camera (not implemented yet)
- Pointcloud Map

### Output
- Pose with Covariance
- Pose Estimator Diagnostics


## Twist Estimator
Twist estimator is a component to estimate ego vehicle twist for precise velocity planning and control. The x-axis velocity and z-axis angular velocity of the vehicle are crucial information. These values are preferable to be noise-free and unbiased.

### Input

- Vehicle CAN
- IMU
- Camera (not implemented yet)

### Output

- Twist with Covariance

## Pose Twist Fusion Filter

### Role

Pose Twist Fusion Filter is a component to integrate the poses estimated by pose estimators and the twists estimated by twist estimators. This assumes sequential Bayesian Filter, such as EKF and particle filter, which calculates vehicle's pose and twist probabilistically. This should also ensure the following functions:
* smoothing of estimated pose (see [TF.md](/design/TF.md))
* outlier rejection of inputs based on previously calculated pose and it's covariance (see [TF.md](/design/TF.md))
* time delay compensation in case pose estimators take time to calculate pose

### Input

- Initial Pose
- Pose with Covariance
- Twist with Covariance

### Output

- Ego Vehicle Pose (/tf from map frame to base_link frame)
- Ego Vehicle Twist

