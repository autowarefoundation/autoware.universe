# map_loader package

This package provides the features of loading various maps.

## pointcloud_map_loader

### Feature

`pointcloud_map_loader` provides pointcloud maps to the other Autoware nodes in various configurations.
Currently, it supports the following four types:
- Publish raw pointcloud map as `sensor_msgs/msg/PointCloud2`
- Publish downsampled pointcloud map as `sensor_msgs/msg/PointCloud2`
- Send partial pointcloud map loading as `autoware_map_msgs/srv/GetPartialPointCloudMap`
- Send differential pointcloud map loading as `autoware_map_msgs/srv/GetDifferentialPointCloudMap`

### Parameters

| Name                          | Type   | Description                                                                        | Default value |
| :---------------------------- | :----- | :--------------------------------------------------------------------------------- | :------------ |
| enable_whole_load             | bool   | A flag to enable raw pointcloud map publishing                                     | true          |
| enable_downsampled_whole_load | bool   | A flag to enable downsampled pointcloud map publishing                             | true          |
| enable_partial_load           | bool   | A flag to enable partial pointcloud map server                                     | true          |
| enable_differential_load      | bool   | A flag to enable differential pointcloud map server                                | true          |
| leaf_size                     | double | Downsampling leaf size (only used when enable_downsampled_whole_load is set true)  | 3.0           |


### Interfaces

- `output/pointcloud_map` (sensor_msgs/msg/PointCloud2) : Raw pointcloud map
- `output/debug/downsampled_pointcloud_map` (sensor_msgs/msg/PointCloud2) : Downsampled pointcloud map
- `service/get_partial_pcd_map` (autoware_map_msgs/srv/GetPartialPointCloudMap) : Partial pointcloud map
- `service/get_differential_pcd_map` (autoware_map_msgs/srv/GetDifferentialPointCloudMap) : Differential pointcloud map

---

## lanelet2_map_loader

### Feature

lanelet2_map_loader loads Lanelet2 file and publishes the map data as autoware_auto_mapping_msgs/HADMapBin message.
The node projects lan/lon coordinates into MGRS coordinates.

### How to run

`ros2 run map_loader lanelet2_map_loader --ros-args -p lanelet2_map_path:=path/to/map.osm`

### Published Topics

- ~output/lanelet2_map (autoware_auto_mapping_msgs/HADMapBin) : Binary data of loaded Lanelet2 Map

---

## lanelet2_map_visualization

### Feature

lanelet2_map_visualization visualizes autoware_auto_mapping_msgs/HADMapBin messages into visualization_msgs/MarkerArray.

### How to Run

`ros2 run map_loader lanelet2_map_visualization`

### Subscribed Topics

- ~input/lanelet2_map (autoware_auto_mapping_msgs/HADMapBin) : binary data of Lanelet2 Map

### Published Topics

- ~output/lanelet2_map_marker (visualization_msgs/MarkerArray) : visualization messages for RViz
