# tensorrt_bevdet

## Purpose

tensorrt_bevdet is a dynamic 3D bev object detection package based on 6 surround view cameras.

## Inner-workings / Algorithms

BEVDet is a BEV perception algorithm based on panoramic cameras. It unifies multi-view images into the perspective of BEV for 3D object detection task. In this implementation, BEVDet network to inference with TensorRT.

## Inputs / Outputs

### Inputs

| Name                   | Type                            | Description                                                 |
| ---------------------- | ------------------------------- | ----------------------------------------------------------- |
| `~/input/pointcloud`   | `sensor_msgs::msg::PointCloud2` | input pointcloud (only used for time alignment and display) |
| `~/input/topic_img_fl` | `sensor_msgs::msg::Image`       | input front_left camera image                               |
| `~/input/topic_img_f`  | `sensor_msgs::msg::Image`       | input front camera image                                    |
| `~/input/topic_img_fr` | `sensor_msgs::msg::Image`       | input front_right camera image                              |
| `~/input/topic_img_bl` | `sensor_msgs::msg::Image`       | input back_left camera image                                |
| `~/input/topic_img_b`  | `sensor_msgs::msg::Image`       | input back camera image                                     |
| `~/input/topic_img_br` | `sensor_msgs::msg::Image`       | input back_right camera image                               |

### Outputs

| Name                  | Type                                             | Description                               |
| --------------------- | ------------------------------------------------ | ----------------------------------------- |
| `~/output/boxes`      | `autoware_perception_msgs::msg::DetectedObjects` | detected objects                          |
| `~/output/pointcloud` | `sensor_msgs::msg::PointCloud2`                  | output pointcloud (only used for display) |

## Limittation

The model is trained on open-source dataset `NuScenes` and has poor generalization on its own dataset, If you want to use this model to infer your data, you need to retrain it.

You can traine models by refer below links.

[BEVDet](https://github.com/HuangJunJie2017/BEVDet/tree/dev2.1)

[BEVDet export onnx](https://github.com/LCH1238/BEVDet/tree/export)
