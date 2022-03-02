# blockage_diag

## Purpose

To ensure the perfomance of LiDAR and safety for autonomous driving, the abnormal condition diagnositcs feature is needed.
LiDAR blockage is abnormal condition of LiDAR when some unwanted objects stitch to and block the light pulses and return signal.
This node's purpose is to detect the existing of blockage on LiDAR and its related size and location.

## Inner-workings / Algorithms

This node bases on the no-return region and its location to decide if it is a blockage.

![blockage situation](./image/blockage_diag.png)

The logic is showed as below

![blockage_diag_flowchart](./image/blockage_diag_flowchart.drawio.svg)

## Inputs / Outputs

This implementation inherits `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Input

| Name                        | Type                            | Description                                                     |
| --------------------------- | ------------------------------- | --------------------------------------------------------------- |
| `~/input/pointcloud_raw_ex` | `sensor_msgs::msg::PointCloud2` | The raw point cloud data is used to detect the no-return region |

### Output

| Name                                                 | Type                                    | Description                                       |
| ---------------------------------------------------- | --------------------------------------- | ------------------------------------------------- |
| `~/output/blockage_diag/debug/blockage_mask_image`   | `sensor_msgs::msg::Image`               | The mask image of detected blockage               |
| `~/output/blockage_diag/debug/ground_blockage_ratio` | `tier4_debug_msgs::msg::Float32Stamped` | The area ratio of blockage region in ground range |
| `~/output/blockage_diag/debug/sky_blockage_ratio`    | `tier4_debug_msgs::msg::Float32Stamped` | The area ratio of blockage region in sky range    |
| `~/output/blockage_diag/debug/lidar_depth_map`       | `sensor_msgs::msg::Image`               | The depth map image of input point cloud          |

## Parameters

| Name                        | Type  | Description                                       |
| --------------------------- | ----- | ------------------------------------------------- |
| `ground_blockage_threshold` | float | The threshold of ground-range blockage area ratio |
| `sky_blockage_threshold`    | float | The threshold of sky-range blockage area ratio    |
| `horizontal_ring_id`        | int   | The id of horizontal ring of the LiDAR            |

## Assumptions / Known limits

The current logic is still limited for dust type of blockage when dust particles are sparsely distributed.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

## (Optional) Future extensions / Unimplemented parts
