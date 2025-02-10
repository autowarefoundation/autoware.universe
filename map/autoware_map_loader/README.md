# autoware_map_loader package

This package provides the features of loading various maps.

## pointcloud_map_loader

### Feature

`pointcloud_map_loader` provides pointcloud maps to the other Autoware nodes in various configurations.
Currently, it supports the following two types:

- Publish raw pointcloud map
- Publish downsampled pointcloud map
- Send partial pointcloud map loading via ROS 2 service
- Send differential pointcloud map loading via ROS 2 service

NOTE: **We strongly recommend to use divided maps when using large pointcloud map to enable the latter two features (partial and differential load). Please go through the prerequisites section for more details, and follow the instruction for dividing the map and preparing the metadata.**

### Prerequisites

#### Prerequisites on pointcloud map file(s)

You may provide either a single .pcd file or multiple .pcd files. If you are using multiple PCD data, it MUST obey the following rules:

1. **The pointcloud map should be projected on the same coordinate defined in `map_projection_loader`**, in order to be consistent with the lanelet2 map and other packages that converts between local and geodetic coordinates. For more information, please refer to [the readme of `map_projection_loader`](https://github.com/autowarefoundation/autoware.universe/tree/main/map/autoware_map_projection_loader/README.md).
2. **It must be divided by straight lines parallel to the x-axis and y-axis**. The system does not support division by diagonal lines or curved lines.
3. **The division size along each axis should be equal.**
4. **The division size should be about 20m x 20m.** Particularly, care should be taken as using too large division size (for example, more than 100m) may have adverse effects on dynamic map loading features in [ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/autoware_ndt_scan_matcher) and [autoware_compare_map_segmentation](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_compare_map_segmentation).
5. **All the split maps should not overlap with each other.**
6. **Metadata file should also be provided.** The metadata structure description is provided below.

#### Metadata structure

The metadata should look like this:

```yaml
x_resolution: 20.0
y_resolution: 20.0
A.pcd: [1200, 2500] # -> 1200 < x < 1220, 2500 < y < 2520
B.pcd: [1220, 2500] # -> 1220 < x < 1240, 2500 < y < 2520
C.pcd: [1200, 2520] # -> 1200 < x < 1220, 2520 < y < 2540
D.pcd: [1240, 2520] # -> 1240 < x < 1260, 2520 < y < 2540
```

where,

- `x_resolution` and `y_resolution`
- `A.pcd`, `B.pcd`, etc, are the names of PCD files.
- List such as `[1200, 2500]` are the values indicate that for this PCD file, x coordinates are between 1200 and 1220 (`x_resolution` + `x_coordinate`) and y coordinates are between 2500 and 2520 (`y_resolution` + `y_coordinate`).

You may use [pointcloud_divider](https://github.com/autowarefoundation/autoware_tools/tree/main/map/autoware_pointcloud_divider) for dividing pointcloud map as well as generating the compatible metadata.yaml.

#### Directory structure of these files

If you only have one pointcloud map, Autoware will assume the following directory structure by default.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
```

If you have multiple rosbags, an example directory structure would be as follows. Note that you need to have a metadata when you have multiple pointcloud map files.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
│ ├── A.pcd
│ ├── B.pcd
│ ├── C.pcd
│ └── ...
├── map_projector_info.yaml
└── pointcloud_map_metadata.yaml
```

### Specific features

#### Publish raw pointcloud map (ROS 2 topic)

The node publishes the raw pointcloud map loaded from the `.pcd` file(s).

#### Publish downsampled pointcloud map (ROS 2 topic)

The node publishes the downsampled pointcloud map loaded from the `.pcd` file(s). You can specify the downsample resolution by changing the `leaf_size` parameter.

#### Publish metadata of pointcloud map (ROS 2 topic)

The node publishes the pointcloud metadata attached with an ID. Metadata is loaded from the `.yaml` file. Please see [the description of `PointCloudMapMetaData.msg`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#pointcloudmapmetadatamsg) for details.

#### Send partial pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given a query from a client node, the node sends a set of pointcloud maps that overlaps with the queried area.
Please see [the description of `GetPartialPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getpartialpointcloudmapsrv) for details.

#### Send differential pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given a query and set of map IDs, the node sends a set of pointcloud maps that overlap with the queried area and are not included in the set of map IDs.
Please see [the description of `GetDifferentialPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getdifferentialpointcloudmapsrv) for details.

#### Send selected pointcloud map (ROS 2 service)

Here, we assume that the pointcloud maps are divided into grids.

Given IDs query from a client node, the node sends a set of pointcloud maps (each of which attached with unique ID) specified by query.
Please see [the description of `GetSelectedPointCloudMap.srv`](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getselectedpointcloudmapsrv) for details.

### Parameters

{{ json_to_markdown("map/autoware_map_loader/schema/pointcloud_map_loader.schema.json") }}

### Interfaces

- `output/pointcloud_map` (sensor_msgs/msg/PointCloud2) : Raw pointcloud map
- `output/pointcloud_map_metadata` (autoware_map_msgs/msg/PointCloudMapMetaData) : Metadata of pointcloud map
- `output/debug/downsampled_pointcloud_map` (sensor_msgs/msg/PointCloud2) : Downsampled pointcloud map
- `service/get_partial_pcd_map` (autoware_map_msgs/srv/GetPartialPointCloudMap) : Partial pointcloud map
- `service/get_differential_pcd_map` (autoware_map_msgs/srv/GetDifferentialPointCloudMap) : Differential pointcloud map
- `service/get_selected_pcd_map` (autoware_map_msgs/srv/GetSelectedPointCloudMap) : Selected pointcloud map
- pointcloud map file(s) (.pcd)
- metadata of pointcloud map(s) (.yaml)

---

## lanelet2_map_loader

### Feature

lanelet2_map_loader loads Lanelet2 file and publishes the map data as autoware_map_msgs/LaneletMapBin message.
The node projects lan/lon coordinates into arbitrary coordinates defined in `/map/map_projector_info` from `map_projection_loader`.
Please see [autoware_map_msgs/msg/MapProjectorInfo.msg](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_map_msgs/msg/MapProjectorInfo.msg) for supported projector types.

### How to run

`ros2 run autoware_map_loader lanelet2_map_loader --ros-args -p lanelet2_map_path:=path/to/map.osm`

### Subscribed Topics

- ~input/map_projector_info (autoware_map_msgs/MapProjectorInfo) : Projection type for Autoware

### Published Topics

- ~output/lanelet2_map (autoware_map_msgs/LaneletMapBin) : Binary data of loaded Lanelet2 Map

### Parameters

{{ json_to_markdown("map/autoware_map_loader/schema/lanelet2_map_loader.schema.json") }}

`use_waypoints` decides how to handle a centerline.
This flag enables to use the `overwriteLaneletsCenterlineWithWaypoints` function instead of `overwriteLaneletsCenterline`. Please see [the document of the autoware_lanelet2_extension package](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md#centerline) in detail.
