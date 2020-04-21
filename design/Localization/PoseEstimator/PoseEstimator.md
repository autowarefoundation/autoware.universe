Pose Estimator
==============

### Role
Pose estimator is a component to estimate ego vehicle pose which includes position and orientation. The final output should also include covariance, which represesents the estimator's confidence on estimated pose. A pose estimator could either be estimate pose on local map, or it can estimate absolute pose using global localizer. The output pose can be publihsed in any frame as long as /tf is provided to project into the "map" frame. Also, pose estimator should stop publishing pose if it is possible to calculate reliability of estimated pose(e.g. matching score with map) and the reliability is low.

## Input

| Input          | Data Type                                            |
|----------------|------------------------------------------------------|
| LiDAR          | `sensor_msgs::PointCoud2`                            |
| GNSS           | `geometry_msgs::PoseWithCovarianceStamped`           |
| Pointcloud Map | `sensor_msgs::PointCoud2`                            |
| Feedback from<br>Pose Twist Fusion Filter | `geometry_msgs::PoseWithCovarianceStamped` |

## Output

| Output         | Data Type                                   | Use Cases of the output         |
|----------------|---------------------------------------------|---------------------------------|
| Initial Pose   | `geometry_msgs::PoseWithCovarianceStamped`  | Pose Twist Fusion Filter        |
| Estimated Pose | `geometry_msgs::PoseWithCovarianceStamped`  | Pose Twist Fusion Filter        |

## Design

This is a sample design of our implementation using NDT Scan Matcher. 
![Pose_Estimator](/design/img/PoseEstimator.svg)

We integrated 3D NDT registration method for sample pose estimation algorithm. The NDT registration method is an local localization method that requires a good intial guess before optimizing pose. In order to realize fully automatic localization, GNSS is used for first initialization. After first loop of pose estimation, the output of pose twist fusion filter is used as next initial guess of NDT registration.

Note that NDT scan matcher does not publish pose when matching score calculated in alignment is less than threshold value to avoid publishing wrong estimated pose to Pose Twist Fusion Filter.

Lidar sensors usually operate at 10 ~ 20 Hz and therefore NDT alignment should be executed within approximately 100 ms. In order to reduce execution time, We apply two pointcloud preprocessors to raw pointcloud from lidar sensors; Crop Measurement Range and DownSampler.
- Crop Measurement Range removes points far from ego vehicle.
- DownSampler reduces the number of points by calculating a centroid of points in each voxelized grid.

Pose initializer adds height information into initial pose obtained from GNSS by looking for minimum height point from points within 1 m. 
