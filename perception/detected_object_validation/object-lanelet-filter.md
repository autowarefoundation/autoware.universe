# object_lanelet_filter

## Purpose

The `object_lanelet_filter` is a node that filters detected object by using vector map.
The objects only inside of the vector map will be published.

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

| Name           | Type | Default Value | Description                                 |
| -------------- | ---- | ------------- | ------------------------------------------- |
| `unknown_only` | bool | true          | If true, only unknown objects are filtered. |

## Assumptions / Known limits

Filtering is performed based on the center position of the object.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
