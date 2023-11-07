# tier4_geo_pose_projector

## Overview 

This node is a simple node that subscribes to the geo-referenced pose topic and publishes the pose in the map frame.

## Subscribed Topics

| Name                            | Type                                             | Description                        |
| ------------------------------- | ------------------------------------------------ | ---------------------------------- |
| `input_geo_pose`                      | `geographic_msgs::msg::GeoPoseWithCovarianceStamped`   | geo-referenced pose      |
| `/map/map_projector_info`           | `tier4_map_msgs::msg::MapProjectedObjectInfo` | map projector info      |

## Published Topics

| Name                            | Type                                             | Description                        |
| ------------------------------- | ------------------------------------------------ | ---------------------------------- |
| `output_pose`                   | `geometry_msgs::msg::PoseWithCovarianceStamped`   | pose in map frame      |

## Parameters

| Parameter             | Type   | Description                      |
| --------------------- | ------ | -------------------------------- |
| `publish_tf`        | bool | publish tf                |
| `parent_frame` | String | parent frame id |
| `child_frame` | String | child frame id |
