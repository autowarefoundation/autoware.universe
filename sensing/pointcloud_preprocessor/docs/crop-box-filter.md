# crop_box_filter

## Purpose

The `crop_box_filter` is a node that removes points with in a given box region. This filter is used to remove the points that hit the vehicle itself.

There are two nodes belongs to this package.

- `crop_box_filter_node`: This node is a ROS2 node that subscribes to a point cloud topic and publishes the filtered point cloud topic.
- `crop_boxes_filter_node`: This node has same functionality as `crop_box_filter_node`, but it can handle multiple crop boxes.

## Inner-workings / Algorithms

Crop box filter is a simple filter that removes points inside a given box.
Remember cropbox coordinate is defined in the `input_frame` frame.

## Inputs / Outputs

This implementation inherit `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherit `pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

For `crop_box_filter_node` and `crop_boxes_filter_node`, the following parameters are used to describe each cropbox.

| Name    | Type   | Default Value | Description                               |
| ------- | ------ | ------------- | ----------------------------------------- |
| `min_x` | double | -1.0          | x-coordinate minimum value for crop range |
| `max_x` | double | 1.0           | x-coordinate maximum value for crop range |
| `min_y` | double | -1.0          | y-coordinate minimum value for crop range |
| `max_y` | double | 1.0           | y-coordinate maximum value for crop range |
| `min_z` | double | -1.0          | z-coordinate minimum value for crop range |
| `max_z` | double | 1.0           | z-coordinate maximum value for crop range |

For multiple crop boxes in `crop_boxes_filter_node`, the following parameters are used to describe each cropbox.

| Name               | Type     | Default Value | Description                                                           |
| ------------------ | -------- | ------------- | --------------------------------------------------------------------- |
| `crop_boxes_names` | string[] | []            | Names of crop boxes                                                   |
| `crop_boxes`       | map      | {}            | Map of crop boxes. Each crop box is described by the above parameters |

Example of `crop_boxes` parameter is:

```yaml
crop_boxes_names: ["right", "left"]
crop_boxes:
  right:
    min_longitudinal_offset: 4.8
    max_longitudinal_offset: 5.3
    min_lateral_offset: -1.3
    max_lateral_offset: -1.05
    min_height_offset: 1.85
    max_height_offset: 2.59
  left:
    min_longitudinal_offset: 4.83
    max_longitudinal_offset: 6.0
    min_lateral_offset: 0.95
    max_lateral_offset: 1.28
    min_height_offset: 1.95
    max_height_offset: 2.6
```

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
