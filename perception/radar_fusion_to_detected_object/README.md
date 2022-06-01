# radar_fusion_to_detected_object

This package is radar-based sensor fusion module to 3d detected object.
Main feature of this package are as following.

- Attach velocity to lidar detection result from radar data to improve for tracking result and planning like adaptive cruise.
- Improve detection result with radar sensor information. If both lidar 3d detected objects with low score and high confidence of radar pointcloud / objects, then improve score of objects.

![process_low_confidence](docs/radar_fusion_to_detected_object_6.drawio.svg)

## Core algorithm

The document of core algorithm is [here](docs/algorithm.md)

### Parameter

#### Radar fusion param

- bounding_box_margin (double): The margin distance from bird's-eye view to choose radar pointcloud/objects within 3D bounding box [m]
  - Default parameter is 2.0
- double split_threshold_velocity (double): The threshold velocity to judge splitting two objects from radar information (now unused) [m/s]
  - Default parameter is 5.0

#### Weight param for velocity estimation

- velocity_weight_average (double): The twist coefficient of average twist of radar data in velocity estimation.
  - Default parameter is 0.0 (= no use)
- velocity_weight_median (double): The twist coefficient of median twist of radar data in velocity estimation.
  - Default parameter is 0.0 (= no use)
- velocity_weight_min_distance (double): The twist coefficient of nearest radar data to the center of bounding box in velocity estimation.
  - Default parameter is 1.0
- velocity_weight_target_value_average (double): The twist coefficient of target value weighted average in velocity estimation.
  - Target value is amplitude if using radar pointcloud.
  - Target value is probability if using radar objects.
  - Default parameter is 0.0 (= no use)
- velocity_weight_target_value_top (double): The twist coefficient of top target value radar data in velocity estimation.
  - Target value is amplitude if using radar pointcloud.
  - Target value is probability if using radar objects.
  - Default parameter is 0.0 (= no use)

#### Parameters for fixed object information

- convert_doppler_to_twist (bool): Convert doppler velocity to twist using the yaw information of a detected object.
  - Default parameter is "false"
- threshold_probability (float): If the probability of an output object is lower than this parameter, and the output object doesn not have radar points/objects, then delete the object.
  - Default parameter is 0.4

## radar_object_fusion_to_detected_object

Sensor fusion with radar objects and a detected object.

- Calculation cost is O(nm).
  - n: the number of radar objects.
  - m: the number of objects from 3d detection.

### How to launch

```sh
roslaunch radar_fusion_to_detected_object radar_object_to_detected_object.launch
```

### Input / Output

- Input
    - `~/input/objects` (autoware_auto_perception_msgs/msg/DetectedObject.msg): 3D detected objects.
    - `~/input/radar_objects` (autoware_auto_perception_msgs/msg/TrackedObjects.msg): Radar objects
- Output
    - `~/output/objects` (autoware_auto_perception_msgs/msg/DetectedObjects.msg): 3D detected object with twist.

### Parameters

- `update_rate_hz` (double): The update rate [hz].
  - Default parameter is 20.0
- Core algorithm parameter

## radar_scan_fusion_to_detected_object (TBD)

TBD
