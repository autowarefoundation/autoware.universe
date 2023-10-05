# front_vehicle_velocity_estimator

This package contains a front vehicle velocity estimation for offline perception module analysis.
This package can:

- Attach velocity to 3D detections from velocity estimation with LiDAR pointcloud.

## Algorithm

- Processing flow
  1. Choose front vehicle from front area objects.
  2. Choose nearest neighbor point within front vehicle.
  3. Estimate velocity of front vehicle by using the differentiated value from time series of nearest neighbor point positions.
  4. Compensate ego vehicle twist

## Input

| Name                 | Type                                                 | Description          |
| -------------------- | ---------------------------------------------------- | -------------------- |
| `~/input/objects`    | autoware_auto_perception_msgs/msg/DetectedObject.msg | 3D detected objects. |
| `~/input/pointcloud` | sensor_msgs/msg/PointCloud2.msg                      | LiDAR pointcloud.    |
| `~/input/odometry`   | nav_msgs::msg::Odometry.msg                          | Odometry data.       |

## Output

| Name                                  | Type                                                  | Description                                   |
| ------------------------------------- | ----------------------------------------------------- | --------------------------------------------- |
| `~/output/objects`                    | autoware_auto_perception_msgs/msg/DetectedObjects.msg | 3D detected object with twist.                |
| `~/debug/nearest_neighbor_pointcloud` | sensor_msgs/msg/PointCloud2.msg                       | The pointcloud msg of nearest neighbor point. |

## Parameter

{{ json_to_markdown("perception/front_vehicle_velocity_estimator/schema/front_vehicle_velocity_estimator.schema.json") }}
