# scan_ground_filter

## Purpose

This node filters the ground points from the pointclouds.

## Inner-workings / Algorithms

This algorithm works by following steps,

1. Divide whole pointclouds into groups by horizontal angle and sort by xy-distance.
2. Check the distance and vertical angle of the point one by one.
3. Set a center of the ground contact point of the rear or front wheels as the initial point.
4. Check vertical angle between the points. If the angle from the initial point is larger than "global_slope_max", the point is classified as "no ground".
5. If the angle from the previous point is larger than "local_max_slope", the point is classified as "no ground".
6. Otherwise the point is labeled as "ground point".
7. If the distance from the last checked point is close, ignore any vertical angle and set current point attribute to the same as the last point.

## Inputs / Outputs

### Input

| Name     | Type                       | Description                 |
| -------- | -------------------------- | --------------------------- |
| `~input` | `sensor_msgs::PointCloud2` | outlier filtered pointcloud |

### Output

| Name      | Type                       | Description          |
| --------- | -------------------------- | -------------------- |
| `~output` | `sensor_msgs::PointCloud2` | no ground pointcloud |

## Parameters

### Node Parameters

| Name                              | Type   | Description                                                                   |
| --------------------------------- | ------ | ----------------------------------------------------------------------------- |
| `base_frame`                      | string | base_link frame                                                               |
| `global_slope_max`                | double | The global angle to classify as the ground or object [deg]                    |
| `local_max_slope`                 | double | The local angle to classify as the ground or object [deg]                     |
| `radial_divider_angle`            | double | The angle which divide the whole pointcloud to sliced group [deg]             |
| `split_points_distance_tolerance` | double | The xy-distance threshold to to distinguishing far and near [m]               |
| `split_height_distance`           | double | The height threshold to distinguishing far and near [m]                       |
| `use_virtual_ground_point`        | bool   | whether to use the ground center of front wheels as the virtual ground point. |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

- Horizontal check for classification is not implemented yet.
- Output ground visibility for diagnostic is not implemented yet.
