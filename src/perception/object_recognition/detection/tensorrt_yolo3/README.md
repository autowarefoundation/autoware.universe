# ROS wrapper for TensorRT YOLOv3

## Referenced repositiory
Please check this repository for detail implementation.
The trained files are provided by the following repository. The trained files are automatically downloaded when you build.

https://github.com/lewes6369/TensorRT-Yolov3

Original URL
- tranined file (416) : 
    https://drive.google.com/drive/folders/18OxNcRrDrCUmoAMgngJlhEglQ1Hqk_NJ

Please note that above repository is under MIT license.
## How to use
1. Build this package.(Automatically download necessary files during build process)
2. `roslaunch tensorrt_yolo3 tensorrt_yolo3.launch`

## Interface
### Input topic type
  `sensor_msgs::Image`
### Output topic type
  `autoware_perception_msgs::DynamicObjectWithFeatureArray`
