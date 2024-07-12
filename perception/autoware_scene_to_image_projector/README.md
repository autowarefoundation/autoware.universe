# scene_to_image_projector

## Purpose

The `scene_to_image_projector` package is designed to visualize 3D data of bounding boxes of detected objects, the trajectory of the ego vehicle, and road boundaries onto the 2D camera image plane. This allows for easy visualization of these elements through a single image topic.

## Inner-working / Algorithms

The algorithm involves several steps to project 3D bounding boxes, the trajectory of the ego vehicle, and road boundaries onto a 2D camera image plane:

1. **Transform 3D Points to Camera Frame**:

   - Convert the 3D coordinates of the bounding boxes, ego vehicle trajectory, and road boundaries from their original coordinate system to the camera's frame of reference.

2. **Project Points onto Image Plane**:

   - Use the camera's 3D projection matrix to project the transformed 3D points onto the 2D image plane.
   - Ensure that the projected points are in front of the camera and not exactly on the image plane.

3. **Connect Projected Points**:
   - Connect the projected 2D points using edges to form the desired shapes such as bounding boxes and trajectory lines.

For handling the bounding boxes:

- Sort the bounding boxes by their distance from the camera to ensure the closest one is drawn first.
- Before drawing each subsequent bounding box, check for overlaps with the already drawn ones.
- Construct a polygon from the existing bounding boxes [1] and perform a point-in-polygon test for each point of the new bounding box.
- If any point of the new bounding box is inside the existing polygon, skip drawing it to prevent overlaps and occlusions. Otherwise, draw the new bounding box.

## Input / Output

### Input

| Name                  | Type                                                                                                | Description                                 |
| --------------------- | --------------------------------------------------------------------------------------------------- | ------------------------------------------- |
| `~/input/image`       | `sensor_msgs::msg::Image`                                                                           | 2D camera image                             |
| `~/input/camera_info` | `sensor_msgs::msg::CameraInfo`                                                                      | Camera calibration and intrinsic parameters |
| `~/input/objects`     | `autoware_perception_msgs::msg::DetectedObjects` or `autoware_perception_msgs::msg::TrackedObjects` | 3D bounding boxes of detected objects       |
| `~/input/trajectory`  | `autoware_planning_msgs::msg::Trajectory`                                                           | Trajectory of the ego vehicle               |
| `~/input/path`        | `autoware_planning_msgs::msg::Path`                                                                 | Road boundaries                             |

### Output

| Name             | Type                      | Description                      |
| ---------------- | ------------------------- | -------------------------------- |
| `~/output/image` | `sensor_msgs::msg::Image` | 2D camera image with projections |

## Parameters

| Name                  | Type   | Description                                        | Default value |
| :-------------------- | :----- | :------------------------------------------------- | :------------ |
| `use_trajectory`      | bool   | Enable visualization of ego vehicle trajectory     | true          |
| `use_road_boundaries` | bool   | Enable visualization of road boundaries            | true          |
| `show_pedestrian`     | bool   | Show pedestrian objects                            | true          |
| `show_bicycle`        | bool   | Show bicycle objects                               | true          |
| `show_motorcycle`     | bool   | Show motorcycle objects                            | true          |
| `show_trailer`        | bool   | Show trailer objects                               | true          |
| `show_bus`            | bool   | Show bus objects                                   | true          |
| `show_truck`          | bool   | Show truck objects                                 | true          |
| `show_car`            | bool   | Show car objects                                   | true          |
| `objects_type`        | string | Type of objects to visualize (tracked or detected) | tracked       |

## Assumptions / Known limits

- This package assumes that the input data is synchronized.
- The accuracy of the projections depends on the calibration of the camera and the precision of the input data.

## References/External links

- [Finding the convex hull of a simple polygon](https://mathweb.ucsd.edu/~ronspubs/83_09_convex_hull.pdf)
- [OpenCV Documentation](https://docs.opencv.org/4.x/)

## Future extensions / Unimplemented parts

- The algorithm works for syncronized data but if the data is unsyncronized another method should be obtained.
- Maximum distance parameter for the projections might be needed.
