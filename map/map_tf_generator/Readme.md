# map_tf_generator

## Purpose

The nodes in this package broadcast the `viewer` frame for visualization of the map in RViz.
The position of the `viewer` frame is the geometric center of input.

Note that there is no module to need the `viewer` frame and this is used only for visualization.

There are two nodes:

- `pcd_map_tf_generator_node` to visualize pointcloud_map
- `vector_map_tf_generator_node` to visualize vector_map

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

#### pcd_map_tf_generator

| Name                  | Type                            | Description                                                       |
| --------------------- | ------------------------------- | ----------------------------------------------------------------- |
| `/map/pointcloud_map` | `sensor_msgs::msg::PointCloud2` | Subscribe pointcloud map to calculate position of `viewer` frames |

#### vector_map_tf_generator

| Name              | Type                                         | Description                                                   |
| ----------------- | -------------------------------------------- | ------------------------------------------------------------- |
| `/map/vector_map` | `autoware_auto_mapping_msgs::msg::HADMapBin` | Subscribe vector map to calculate position of `viewer` frames |

### Output

| Name         | Type                     | Description               |
| ------------ | ------------------------ | ------------------------- |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Broadcast `viewer` frames |

## Parameters

### Node Parameters

None

### Core Parameters

| Name           | Type   | Default Value | Explanation                           |
| -------------- | ------ | ------------- | ------------------------------------- |
| `viewer_frame` | string | viewer        | Name of `viewer` frame                |
| `map_frame`    | string | map           | The parent frame name of viewer frame |

## Assumptions / Known limits

TBD.
