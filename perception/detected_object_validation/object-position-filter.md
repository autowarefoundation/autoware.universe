# object_lanelet_filter

## Purpose

The `object_lanelet_filter` is a node that filters detected object based on x,y values.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name               | Type                                                  | Description           |
| ------------------ | ----------------------------------------------------- | --------------------- |
| `input/vector_map` | `autoware_auto_mapping_msgs::msg::HADMapBin`          | vector map            |
| `input/object`     | `autoware_auto_perception_msgs::msg::DetectedObjects` | input detected object |

### Output

| Name            | Type                                                  | Description              |
| --------------- | ----------------------------------------------------- | ------------------------ |
| `output/object` | `autoware_auto_perception_msgs::msg::DetectedObjects` | filtered detected object |

## Parameters

### Core Parameters

| Name                    | Type  | Default Value | Description                                                                                                 |
| ----------------------- | ----- | ------------- | ----------------------------------------------------------------------------------------------------------- |
| `unknown_only`          | bool  | true          | If true, only unknown objects are filtered.                                                                 |
| `upper_bound_x`         | float | 100.00        | Bound for filtering. Only used if filter_by_xy_position is true                                             |
| `lower_bound_x`         | float | 0.00          | Bound for filtering. Only used if filter_by_xy_position is true                                             |
| `upper_bound_y`         | float | 50.00         | Bound for filtering. Only used if filter_by_xy_position is true                                             |
| `lower_bound_y`         | float | -50.00        | Bound for filtering. Only used if filter_by_xy_position is true                                             |

## Assumptions / Known limits

Filtering is performed based on the center position of the object.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
