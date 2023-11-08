# tier4_geo_pose_projector

## Overview

This node is a simple node that subscribes to the geo-referenced pose topic and publishes the pose in the map frame.

## Subscribed Topics

| Name                      | Type                                                 | Description         |
| ------------------------- | ---------------------------------------------------- | ------------------- |
| `input_geo_pose`          | `geographic_msgs::msg::GeoPoseWithCovarianceStamped` | geo-referenced pose |
| `/map/map_projector_info` | `tier4_map_msgs::msg::MapProjectedObjectInfo`        | map projector info  |

## Published Topics

| Name          | Type                                            | Description       |
| ------------- | ----------------------------------------------- | ----------------- |
| `output_pose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | pose in map frame |

## Parameters

| Parameter      | Type   | Description     |
| -------------- | ------ | --------------- |
| `publish_tf`   | bool   | publish tf      |
| `parent_frame` | String | parent frame id |
| `child_frame`  | String | child frame id  |

## Limitations

The covariance conversion may be incorrect depending on the projection type you are using. The covariance of input topic is expressed in (Latitude, Longitude, Altitude) as a diagonal matrix. 
Currently, we assume that the x axis is the east direction and the y axis is the north direction. Thus, the conversion may be incorrect when this assumption breaks, especially when the covariance of latitude and longitude is different.
