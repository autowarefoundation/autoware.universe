# elevation_map_loader

## Purpose

This package provides elevation map for compare_map_segmentation.

## Inner-workings / Algorithms

Generate elevation_map from subscribed pointcloud_map and vector_map and publish it.
Save the generated elevation_map locally and load it from next time.

The elevation value of each cell is the average value of z of the points of the lowest cluster.  
Cells with No elevation value can be inpainted using the values of neighboring cells.

<p align="center">
  <img src="./media/elevation_map.png" width="1500">
</p>

## Inputs / Outputs

### Input

| Name                            | Type                                            | Description                                |
| ------------------------------- | ----------------------------------------------- | ------------------------------------------ |
| `input/pointcloud_map`          | `sensor_msgs::msg::PointCloud2`                 | The point cloud map                        |
| `input/vector_map`              | `autoware_auto_mapping_msgs::msg::HADMapBin`    | (Optional) The binary data of lanelet2 map |
| `input/pointcloud_map_metadata` | `autoware_map_msgs::msg::PointCloudMapMetaData` | (Optional) The metadata of point cloud map |

### Output

| Name                         | Type                            | Description                                                          |
| ---------------------------- | ------------------------------- | -------------------------------------------------------------------- |
| `output/elevation_map`       | `grid_map_msgs::msg::GridMap`   | The elevation map                                                    |
| `output/elevation_map_cloud` | `sensor_msgs::msg::PointCloud2` | (Optional) The point cloud generated from the value of elevation map |

### Service

| Name                           | Type                                               | Description                                                                                                                               |
| ------------------------------ | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `service/get_selected_pcd_map` | `autoware_map_msgs::srv::GetSelectedPointCloudMap` | (Optional) service to request point cloud map. If pointcloud_map_loader uses selected pointcloud map loading via ROS 2 service, use this. |

## Parameters

{{ json_to_markdown("perception/elevation_map_loader/schema/elevation_map_loader.schema.json") }}
