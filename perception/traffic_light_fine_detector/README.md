# traffic_light_fine_detector

## Purpose

It is a package for traffic light detection using YoloX-s.

## Training Information

### Pretrained Model

The model is based on [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) and the pretrained model could be downloaded from [here](https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.pth).

### Training Data

The model was fine-tuned on around 17,000 TIER IV internal images of Japanese traffic lights.

### Trained Onnx model

- <https://drive.google.com/uc?id=1USFDPRH9JrVdGoqt27qHjRgittwc0kcO>

## Inner-workings / Algorithms

Based on the camera image and the global ROI array detected by `map_based_detection` node, a CNN-based detection method enables highly accurate traffic light detection.

## Inputs / Outputs

### Input

| Name            | Type                                                       | Description                                      |
| --------------- | ---------------------------------------------------------- | ------------------------------------------------ |
| `~/input/image` | `sensor_msgs/Image`                                        | The full size camera image                       |
| `~/input/rois`  | `autoware_auto_perception_msgs::msg::TrafficLightRoiArray` | The array of ROIs detected by map_based_detector |

### Output

| Name                  | Type                                                       | Description                  |
| --------------------- | ---------------------------------------------------------- | ---------------------------- |
| `~/output/rois`       | `autoware_auto_perception_msgs::msg::TrafficLightRoiArray` | The detected accurate rois   |
| `~/debug/exe_time_ms` | `tier4_debug_msgs::msg::Float32Stamped`                    | The time taken for inference |

## Parameters

### Core Parameters

| Name                          | Type   | Default Value | Description                                                            |
| ----------------------------- | ------ | ------------- | ---------------------------------------------------------------------- |
| `fine_detection_score_thresh` | double | 0.3           | If the objectness score is less than this value, the object is ignored |
| `fine_detection_nms_thresh`   | double | 0.65          | IoU threshold to perform Non-Maximum Suppression                       |

### Node Parameters

| Name                        | Type   | Default Value                 | Description                                                        |
| --------------------------- | ------ | ----------------------------- | ------------------------------------------------------------------ |
| `fine_detection_onnx_file`  | string | "./data/tlr_yolox-s_nms.onnx" | The onnx file name for yolo model                                  |
| `fine_detection_label_file` | string | "./data/voc_labels_tl.txt"    | The label file with label names for detected objects written on it |
| `fine_detector_precision`   | string | "FP32"                        | The inference mode: "FP32", "FP16", "INT8"                         |
| `approximate_sync`          | bool   | false                         | Flag for whether to ues approximate sync policy                    |

## Assumptions / Known limits

## Reference repositories

YOLOX github repository

- <https://github.com/Megvii-BaseDetection/YOLOX>
