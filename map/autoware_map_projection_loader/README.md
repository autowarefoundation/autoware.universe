# autoware_map_projection_loader

## Feature

`autoware_map_projection_loader` is responsible for publishing `map_projector_info` that defines in which kind of coordinate Autoware is operating.
This is necessary information especially when you want to convert from global (geoid) to local coordinate or the other way around.

- If `map_projector_info_path` DOES exist, this node loads it and publishes the map projection information accordingly.
- If `map_projector_info_path` does NOT exist, the node assumes that you are using the `MGRS` projection type, and loads the lanelet2 map instead to extract the MGRS grid.
  - **DEPRECATED WARNING: This interface that uses the lanelet2 map is not recommended. Please prepare the YAML file instead.**

## Map projector info file specification

You need to provide a YAML file, namely `map_projector_info.yaml` under the `map_path` directory. For `pointcloud_map_metadata.yaml`, please refer to the Readme of `autoware_map_loader`.

```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
├── map_projector_info.yaml
└── pointcloud_map_metadata.yaml
```

There are three types of transformations from latitude and longitude to XYZ coordinate system as shown in the figure below. Please refer to the following details for the necessary parameters for each projector type.

![node_diagram](docs/map_projector_type.svg)

### Using local coordinate

```yaml
# map_projector_info.yaml
projector_type: Local
```

#### Limitation

The functionality that requires latitude and longitude will become unavailable.

The currently identified unavailable functionalities are:

- GNSS localization
- Sending the self-position in latitude and longitude using ADAPI

### Using MGRS

If you want to use MGRS, please specify the MGRS grid as well.

```yaml
# map_projector_info.yaml
projector_type: MGRS
vertical_datum: WGS84
mgrs_grid: 54SUE
```

#### Limitation

It cannot be used with maps that span across two or more MGRS grids. Please use it only when it falls within the scope of a single MGRS grid.

### Using LocalCartesianUTM

If you want to use local cartesian UTM, please specify the map origin as well.

```yaml
# map_projector_info.yaml
projector_type: LocalCartesianUTM
vertical_datum: WGS84
map_origin:
  latitude: 35.6762 # [deg]
  longitude: 139.6503 # [deg]
  altitude: 0.0 # [m]
```

### Using TransverseMercator

If you want to use Transverse Mercator projection, please specify the map origin as well.

```yaml
# map_projector_info.yaml
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 35.6762 # [deg]
  longitude: 139.6503 # [deg]
  altitude: 0.0 # [m]
```

## Published Topics

- `~/map_projector_info` (autoware_map_msgs/MapProjectorInfo) : This topic shows the definition of map projector information

## Parameters

Note that these parameters are assumed to be passed from launch arguments, and it is not recommended to directly write them in `map_projection_loader.param.yaml`.

{{ json_to_markdown("map/autoware_map_projection_loader/schema/map_projection_loader.schema.json") }}
