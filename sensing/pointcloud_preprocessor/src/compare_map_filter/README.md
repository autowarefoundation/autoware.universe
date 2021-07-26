# compare_elevation_map_filter

## Role

Filter the ground points from the input pointcloud using an elevation map.

## Feature details

Compare the z of the input points with the value of elevation_map. The height difference is calculated by the binary integration of neighboring cells. Remove points whose height difference is below the `height_diff_thresh`.

<p align="center">
  <img src="./media/compare_elevation_map.png" width="1000">
</p>

## Node

### Topic

#### Subscribed Topic

- **input** `[sensor_msgs::msg::PointCloud2]`
  - input sensor pointcloud
- **input/elevation_map** `[grid_map_msgs::msg::GridMap]`
  - elevation map

#### Published Topic

- **output** `[sensor_msgs::msg::PointCloud2]`
  - filtered point cloud

### TF

#### Published TF

- None

#### Subscribed TF

- input point cloud frame to elevation map frame
  - ex) `/base_link` to `/map`
- elevation map frame to input point cloud frame
  - ex) `/map` to `/base_link`

### Parameter description

| Name               | Type        | Description                                                                     | Default value |
| :----------------- | :---------- | :------------------------------------------------------------------------------ | :------------ |
| map_layer_name     | std::string | elevation map layer name                                                        | elevation     |
| map_frame          | float       | frame_id of the map that is temporarily used before elevation_map is subscribed | map           |
| height_diff_thresh | float       | Remove points whose height difference is below this value [m]                   | 0.15          |
