# map_projection_loader

## Feature

`map_projection_loader` is responsible for publishing `map_projector_info` that defines in which kind of coordinate the Autoware is operating on.
This is a necessary information especially when you want to convert from global (geoid) to local coordinate or the other way around.

- If `map_projector_info_path` DOES exist, this node loads it and publish the map projection information accordingly.
- If `map_projector_info_path` does NOT exist, the node assumes that you are using `MGRS` projection type, and loads lanelet2 map instead to extract MGRS grid.
  - **DEPRECATED WARNING: This interface that uses lanelet2 map is not recommended. Please prepare YAML file instead.**

## Map projector info file specification

You need to provide a YAML file, namely `map_projector_info.yaml` under the `map_path` directory. For `pointcloud_map_metadata.yaml`, please refer to the Readme of `map_loader`.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
├── map_projector_info.yaml
└── pointcloud_map_metadata.yaml
```

### Using local coordinate

```yaml
type: "Local"
```

### Using MGRS

If you want to use MGRS, please specify MGRS grid as well.

```yaml
type: "MGRS"
mgrs_grid: "54SUE"
```

### Using UTM

If you want to use UTM, please specify the map origin as well.

```
type: "UTM"
map_origin:
  latitude: 35.6092
  longitude: 139.7303
  altitude: 0.0
```

### Using Transverse Mercator

If you want to use Transverse Mercator projection, please specify the map origin as well.

```
type: "TransverseMercator"
map_origin:
  latitude: 36.1148
  longitude: 137.9532
  altitude: 0.0
```

## Published Topics

- ~/map_projector_info (tier4_map_msgs/MapProjectorInfo) : Topic for defining map projector information

## Parameters

| Name                    | Type        | Description                                                                      |
| :---------------------- | :---------- | :------------------------------------------------------------------------------- |
| map_projector_info_path | std::string | A path to map_projector_info.yaml (used by default)                              |
| lanelet2_map_path       | std::string | A path to lanelet2 map (used only when `map_projector_info_path` does not exist) |
