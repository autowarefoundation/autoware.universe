# scene_to_image_projector

## Purpose

The `scene_to_image_projector` package is designed to visualize 3D data of bounding boxes of detected objects, the trajectory of the ego vehicle, and road boundaries onto the 2D camera image plane. This allows for easy visualization of these elements through a single image topic.

## Inner-working / Algorithms

Explain:
1- 3D to 2D point projection onto the image
2- Convex hull algorithm
3- How to discard overlapping bounding boxes. 

## Input / Output

### Input

| Name                        | Type                                                                                               | Description                             |
| --------------------------- | ---------------------------------------------------------------------------------------------------| --------------------------------------- |
| `~/input/image`             | `sensor_msgs::msg::Image`                                                                          | 2D camera image                         |
| `~/input/camera_info`       | `sensor_msgs::msg::CameraInfo`                                                                     | Camera calibration and intrinsic parameters |
| `~/input/objects`           | `autoware_perception_msgs::msg::DetectedObjects` or `autoware_perception_msgs::msg::TrackedObjects`| 3D bounding boxes of detected objects   |
| `~/input/trajectory`        | `autoware_planning_msgs::msg::Trajectory`                                                          | Trajectory of the ego vehicle           |
| `~/input/path`              | `autoware_planning_msgs::msg::Path`                                                                | Road boundaries            |

### Output

| Name                 | Type                                   | Description                              |
| -------------------- | -------------------------------------- | ---------------------------------------- |
| `~/output/image`       | `sensor_msgs::msg::Image`              | 2D camera image with projections         |

## Parameters

| Name                    | Type    | Description                                    | Default value |
| :---------------------- | :------ | :--------------------------------------------- | :------------ |
| `use_trajectory`        | bool    | Enable visualization of ego vehicle trajectory | true          |
| `use_road_boundaries`   | bool    | Enable visualization of road boundaries        | true          |
| `show_pedestrian`       | bool    | Show pedestrian objects                        | true          |
| `show_bicycle`          | bool    | Show bicycle objects                           | true          |
| `show_motorcycle`       | bool    | Show motorcycle objects                        | true          |
| `show_trailer`          | bool    | Show trailer objects                           | true          |
| `show_bus`              | bool    | Show bus objects                               | true          |
| `show_truck`            | bool    | Show truck objects                             | true          |
| `show_car`              | bool    | Show car objects                               | true          |
| `objects_type`          | string  | Type of objects to visualize (tracked or detected) | tracked       |

## Assumptions / Known limits

- This package assumes that the input data is synchronized and 
- The accuracy of the projections depends on the calibration of the camera and the precision of the input data.

## (Optional) Performance characterization

### Processing time

- Average processing time per frame: X ms
- Maximum processing time per frame: Y ms

## (Optional) References/External links

- [1] {link_to_relevant_paper}  Convex Hull Paper
- [2] {link_to_related_project} 3D point projection paper 
- [3] {link_to_related_paper}   opencv documentation

## (Optional) Future extensions / Unimplemented parts

- Adaptation for non-sync data ? explain more 