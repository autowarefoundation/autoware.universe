# dora-yolo目标检测

## 依赖项

1. pytroch 和 troch vision   (需提前在电脑上安装)
2. 安装opencv 环境  `pip install numpy opencv-python pyarrow`

## 启动检测

```
dora up
dora start dataflow_yolo.yaml --attach --hot-reload --name yolo
```

1.可以通过 dataflow_yolo.yaml 选择使用yolov5或yoloV8模型 ，

-  object_detection_yolov5.py 表示使用yolov5算法检测目标   
-  object_detection_yolov8.py 表示使用yolov8算法检测目标

2.可通过修改 webcam.py 中的 CAMERA_INDEX 参数选择不同的相机

