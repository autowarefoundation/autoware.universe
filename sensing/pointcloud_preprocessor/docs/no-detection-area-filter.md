# no_detection_area_filter

## Purpose

The `no_detection_area_filter` is a node that removes points inside the area tagged as `no_detection_area` in lanelet map.

## Inner-workings / Algorithms

- Extract the `no_detection_area` lanelets that intersects with the bounding box of input points
- Create the 2D polygon from the extracted lanelet polygons
- Remove input points inside the polygon by `boost::geometry::within()`

![no_detection_area_figure](./image/no_detection_area_filter-overview.svg)

## Inputs / Outputs

This implementation inherit `pointcloud_preprocessor::Filter` class, please see also [README](../README.md).

### Input

| Name                 | Type                                         | Description                            |
| -------------------- | -------------------------------------------- | -------------------------------------- |
| `~/input`            | `sensor_msgs::msg::PointCloud2`              | input points                           |
| `~/input/vector_map` | `autoware_auto_mapping_msgs::msg::HADMapBin` | lanelet2 map used for filtering points |

### Output

| Name       | Type                            | Description     |
| ---------- | ------------------------------- | --------------- |
| `~/output` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Core Parameters

## Assumptions / Known limits
