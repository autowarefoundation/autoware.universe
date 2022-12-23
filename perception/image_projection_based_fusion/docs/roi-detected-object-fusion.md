# roi_detected_object_fusion

## Purpose

The `roi_detected_object_fusion` is a package to overwrite labels of detected objects with that of Region Of Interests (ROIs) by a 2D object detector.

## Inner-workings / Algorithms

The algorithm is shown below and see `Parameters` for the meaning of each parameter :

1. If `existence_probability` of detected objects is greater than the threshold, they passthrough without doing anything.
2. The rest of detected objects are projected onto image planes, and if ROIs are overlapped with them, they are output as fused objects. Intersection over Union (IoU) is used to determine if there are overlaps between them.

The `DetectedObject` has three shape and the polygon vertices of a object are as below:

- `BOUNDING_BOX`: The 8 corners of a bounding box.
- `CYLINDER`: The circle is approximated by a hexagon.
- `POLYGON`: Not implemented yet.

## Inputs / Outputs

### Input

| Name                     | Type                                                     | Description                                                |
| ------------------------ | -------------------------------------------------------- | ---------------------------------------------------------- |
| `input`                  | `autoware_auto_perception_msgs::msg::DetectedObjects`    | input detected objects                                     |
| `input/camera_info[0-7]` | `sensor_msgs::msg::CameraInfo`                           | camera information to project 3d points onto image planes. |
| `input/rois[0-7]`        | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ROIs from each image.                                      |
| `input/image_raw[0-7]`   | `sensor_msgs::msg::Image`                                | images for visualization.                                  |

### Output

| Name                      | Type                                                  | Description                |
| ------------------------- | ----------------------------------------------------- | -------------------------- |
| `output`                  | `autoware_auto_perception_msgs::msg::DetectedObjects` | detected objects           |
| `~/debug/image_raw[0-7]`  | `sensor_msgs::msg::Image`                             | images for visualization,  |
| `~/debug/fused_objects`   | `autoware_auto_perception_msgs::msg::DetectedObjects` | fused detected objects     |
| `~/debug/ignored_objects` | `autoware_auto_perception_msgs::msg::DetectedObjects` | not fused detected objects |

## Parameters

### Core Parameters

| Name                                            | Type   | Description                                                                                                  |
| ----------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------ |
| `rois_number`                                   | int    | the number of input rois                                                                                     |
| `debug_mode`                                    | bool   | If `true`, subscribe and publish images for visualization.                                                   |
| `use_iou`                                       | bool   | calculate IoU both along x-axis and y-axis                                                                   |
| `min_iou_threshold`                             | double | If the iou between detected objects and rois is greater than threshold, the objects are classified as fused. |
| `passthrough_lower_bound_probability_threshold` | double | If `existence_probability` of detected objects is greater than the threshold, they passthrough as outputs    |
| `use_roi_probability`                           | float  | use `existence_probability` of ROIs to match with detected objects                                           |
| `roi_probability_threshold`                     | double | If `existence_probability` of ROIs is greater than the threshold, matched detected objects are output        |

## Assumptions / Known limits

`POLYGON`, which is a shape of a detected object, isn't supported yet.
