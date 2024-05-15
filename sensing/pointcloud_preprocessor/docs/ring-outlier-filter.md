# ring_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain.

## Inner-workings / Algorithms

A method of operating scan in chronological order and removing noise based on the rate of change in the distance between points

![ring_outlier_filter](./image/outlier_filter-ring.drawio.svg)

Another feature of this node is that it calculates visibility based on outlier pointcloud and publish score as a topic. With this function, for example, in heavy rain, the sensing module can notify that the processing performance has reached its limit, which can lead to ensuring the safety of the vehicle.

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

| Name                         | Type    | Default Value | Description                                                                                                                   |
| ---------------------------- | ------- | ------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `distance_ratio`             | double  | 1.03          |                                                                                                                               |
| `object_length_threshold`    | double  | 0.1           |                                                                                                                               |
| `num_points_threshold`       | int     | 4             |                                                                                                                               |
| `max_rings_num`              | uint_16 | 128           |                                                                                                                               |
| `max_points_num_per_ring`    | size_t  | 4000          | Set this value large enough such that `HFoV / resolution < max_points_num_per_ring`                                           |
| `publish_outlier_pointcloud` | bool    | false         | Flag to publish outlier pointcloud and visibility score. Due to performance concerns, please set to false during experiments. |
| `min_azimuth_deg`            | float   | 0.0           | The left limit of azimuth for visibility score calculation                                                                    |
| `max_azimuth_deg`            | float   | 360.0         | The right limit of azimuth for visibility score calculation                                                                   |
| `max_distance`               | float   | 12.0          | The limit distance for visibility score calculation                                                                           |
| `vertical_bins`              | int     | 128           | The number of vertical bin for visibility histogram                                                                           |
| `horizontal_bins`            | int     | 36            | The number of horizontal bin for visibility histogram                                                                         |
| `noise_threshold`            | int     | 2             | The parameter for determining whether it is noise                                                                             |

## Assumptions / Known limits

It is a prerequisite to input a scan point cloud in chronological order. In this repository it is defined as blow structure (please refer to [PointXYZIRADT](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/5d8dff0db51634f0c42d2a3e87ca423fbee84348/sensing/preprocessor/pointcloud/pointcloud_preprocessor/include/pointcloud_preprocessor/outlier_filter/ring_outlier_filter_nodelet.hpp#L53-L62)).

- X: x
- Y: y
- z: z
- I: intensity
- R: ring
- A :azimuth
- D: distance
- T: time_stamp

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
