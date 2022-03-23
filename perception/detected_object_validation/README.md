# detected_object_validation

## Purpose

The purpose of this package is to eliminate obvious false positives of DetectedObjects.

# occupancy grid based validator
## Inner-workings / Algorithms

Compare the occupancy grid map with the DetectedObject, and if a larger percentage of obstacles are in freespace, delete them.

![debug sample image](image/occupancy_grid_based_validator/debug_image.png)

Basically, it takes an occupancy grid map as input and generates a binary image of freespace or other.

A mask image is generated for each DetectedObject and the average value (percentage) in the mask image is calculated.
If the percentage is low, it is deleted.

## Inputs / Outputs

### Input

| Name                         | Type                                                  | Description                                                 |
| ---------------------------- | ----------------------------------------------------- | ----------------------------------------------------------- |
| `~/input/detected_objects`   | `autoware_auto_perception_msgs::msg::DetectedObjects` | DetectedObjects                                             |
| `~/input/occupancy_grid_map` | `nav_msgs::msg::OccupancyGrid`                        | OccupancyGrid with no time series calculation is preferred. |

### Output

| Name               | Type                                                  | Description               |
| ------------------ | ----------------------------------------------------- | ------------------------- |
| `~/output/objects` | `autoware_auto_perception_msgs::msg::DetectedObjects` | validated DetectedObjects |

## Parameters

| Name             | Type  | Description                                        |
| ---------------- | ----- | -------------------------------------------------- |
| `mean_threshold` | float | The percentage threshold of allowed non-freespace. |
| `enable_debug`   | bool  | Whether to display debug images or not?            |

## Assumptions / Known limits

Currently, only vehicle represented as BoundingBox are supported.

# obstacle pointcloud based validator

## Inner-workings / Algorithms

If the number of obstacle point groups in the DetectedObjects is small, it is considered a false positive and removed.
The obstacle point cloud can be a point cloud after compare map filtering or a ground filtered point cloud.


![debug sample image](image/obstacle_pointcloud_based_validator/debug_image.gif)

In the debug image above, the red DetectedObject is the validated object. The blue object is the deleted object.

## Inputs / Outputs

### Input

| Name                         | Type                                                  | Description                                                 |
| ---------------------------- | ----------------------------------------------------- | ----------------------------------------------------------- |
| `~/input/detected_objects`   | `autoware_auto_perception_msgs::msg::DetectedObjects` | DetectedObjects                                             |
| `~/input/obstacle_pointcloud` | `sensor_msgs::msg::PointCloud2`                        | Obstacle point cloud of dynamic objects |

### Output

| Name               | Type                                                  | Description               |
| ------------------ | ----------------------------------------------------- | ------------------------- |
| `~/output/objects` | `autoware_auto_perception_msgs::msg::DetectedObjects` | validated DetectedObjects |

## Parameters

| Name             | Type  | Description                                        |
| ---------------- | ----- | -------------------------------------------------- |
| `min_pointcloud_num` | float | Threshold for the minimum number of obstacle point clouds in DetectedObjects |
| `enable_debugger`   | bool  | Whether to create debug topics or not?            |

## Assumptions / Known limits

Currently, only represented objects as BoundingBox or Cylinder are supported.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
