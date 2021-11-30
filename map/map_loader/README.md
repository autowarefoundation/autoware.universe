# map_loader package

This package provides the features of loading various maps.

## pointcloud_map_loader

### Feature

pointcloud_map_loader loads PointCloud file and publish the map data as sensor_msgs/PointCloud2 message.

### How to run

`rosrun map_loader pointcloud_map_loader path/to/pointcloud1.pcd path/to/pointcloud2.pcd ...`

### Published Topics

- pointcloud_map (sensor_msgs/PointCloud2) : PointCloud Map

---

## lanelet2_map_loader

### Feature

lanelet2_map_loader loads Lanelet2 file and publish the map data as autoware_lanelet2_msgs/MapBin message.
The node projects lan/lon coordinates into MGRS coordinates.

### How to run

`rosrun map_loader lanelet2_map_loader path/to/map.osm`

### Published Topics

- ~output/lanelet2_map (autoware_lanelet2_msgs/MapBin) : Binary data of loaded Lanelet2 Map

---

## lanelet2_map_visualization

### Feature

lanelet2_map_visualization visualizes autoware_lanelet2_msgs/MapBin messages into visualization_msgs/MarkerArray.

### How to Run

`rosrun map_loader lanelet2_map_visualization`

### Subscribed Topics

- ~input/lanelet2_map (autoware_lanelet2_msgs/MapBin) : binary data of Lanelet2 Map

### Published Topics

- ~output/lanelet2_map_marker (visualization_msgs/MarkerArray) : visualization messages for RVIZ
