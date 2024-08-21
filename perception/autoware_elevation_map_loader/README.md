# autoware_elevation_map_loader

## Purpose

This package provides elevation map for autoware_compare_map_segmentation.

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
| `input/vector_map`              | `autoware_map_msgs::msg::LaneletMapBin`         | (Optional) The binary data of lanelet2 map |
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

### Node Parameters

{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/node.json") }}

### Grid Map Parameters

See: <https://github.com/ANYbotics/grid_map/tree/ros2/grid_map_pcl>

{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/grid_map.json") }}

### Point Cloud Preprocessing Parameters

#### Rigid body transform parameters

Rigid body transform that is applied to the point cloud before computing elevation.

{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/cloud_transform_translation.json") }}
{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/cloud_transform_rotation.json") }}

#### Cluster Extraction Parameters

Cluster extraction is based on pcl algorithms. See <https://pointclouds.org/documentation/tutorials/cluster_extraction.html> for more details.

{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/cluster_extraction.json") }}

#### Outlier Removal Parameters

See <https://pointclouds.org/documentation/tutorials/statistical_outlier.html> for more explanation on outlier removal.

{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/outlier_removal.json") }}

#### Subsampling Parameters

See <https://pointclouds.org/documentation/tutorials/voxel_grid.html> for more explanation on point cloud downsampling.

{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/downsampling.json") }}
{{ json_to_markdown("perception/autoware_elevation_map_loader/schema/sub/downsampling_voxel_size.json") }}
