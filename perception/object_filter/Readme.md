# detected_object_filter

## Purpose

The `detected_object_filter` is a node that filters detected object based on x,y values, or by using vector map.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name               | Type                                                  | Description           |
| ------------------ | ----------------------------------------------------- | --------------------- |
| `input/vector_map` | `autoware_auto_mapping_msgs::msg::HADMapBin`          | vector map            |
| `input/object`     | `autoware_auto_perception_msgs::msg::DetectedObjects` | input detected object |

### Output

| Name            | Type                            | Description              |
| --------------- | ------------------------------- | ------------------------ |
| `output/object` | `sensor_msgs::msg::PointCloud2` | filtered detected object |

## Parameters

### Core Parameters

| Name                    | Type  | Default Value | Description                                                                                                 |
| ----------------------- | ----- | ------------- | ----------------------------------------------------------------------------------------------------------- |
| `filter_by_xy_position` | bool  | false         | If true, objects are filtered based on x,y value; if set to false, vector map based filtering is performed. |
| `upper_bound_x`         | float | 100.00        | Bound for filtering. Only used if filter_by_xy_position is true                                             |
| `lower_bound_x`         | float | 0.00          | Bound for filtering. Only used if filter_by_xy_position is true                                             |
| `upper_bound_y`         | float | 0.00          | Bound for filtering. Only used if filter_by_xy_position is true                                             |
| `lower_bound_y`         | float | 0.00          | Bound for filtering. Only used if filter_by_xy_position is true                                             |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
