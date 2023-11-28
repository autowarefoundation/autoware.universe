# traffic_light_fine_detector

## Purpose

It is a package for traffic light detection using YoloX-s.

## Training Information

### Pretrained Model

The model is based on [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) and the pretrained model could be downloaded from [here](https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.pth).

### Training Data

The model was fine-tuned on around 17,000 TIER IV internal images of Japanese traffic lights.

### Trained Onnx model

You can download the ONNX file using these instructions.  
Please visit [autoware-documentation](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/models/index.md) for more information.

## Inner-workings / Algorithms

Based on the camera image and the global ROI array detected by `map_based_detection` node, a CNN-based detection method enables highly accurate traffic light detection.

## Inputs / Outputs

### Input

| Name            | Type                                               | Description                                                         |
| --------------- | -------------------------------------------------- | ------------------------------------------------------------------- |
| `~/input/image` | `sensor_msgs/Image`                                | The full size camera image                                          |
| `~/input/rois`  | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The array of ROIs detected by map_based_detector                    |
| `~/expect/rois` | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The array of ROIs detected by map_based_detector without any offset |

### Output

| Name                  | Type                                               | Description                  |
| --------------------- | -------------------------------------------------- | ---------------------------- |
| `~/output/rois`       | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The detected accurate rois   |
| `~/debug/exe_time_ms` | `tier4_debug_msgs::msg::Float32Stamped`            | The time taken for inference |

## Parameters

### Core Parameters

| Name                         | Type   | Default Value | Description                                                            |
| ---------------------------- | ------ | ------------- | ---------------------------------------------------------------------- |
| `fine_detector_score_thresh` | double | 0.3           | If the objectness score is less than this value, the object is ignored |
| `fine_detector_nms_thresh`   | double | 0.65          | IoU threshold to perform Non-Maximum Suppression                       |

### Node Parameters

| Name                       | Type   | Default Value               | Description                                                        |
| -------------------------- | ------ | --------------------------- | ------------------------------------------------------------------ |
| `data_path`                | string | "$(env HOME)/autoware_data" | packages data and artifacts directory path                         |
| `fine_detector_model_path` | string | ""                          | The onnx file name for yolo model                                  |
| `fine_detector_label_path` | string | ""                          | The label file with label names for detected objects written on it |
| `fine_detector_precision`  | string | "fp32"                      | The inference mode: "fp32", "fp16"                                 |
| `approximate_sync`         | bool   | false                       | Flag for whether to ues approximate sync policy                    |

## Training Traffic Light Fine Detector model

### Overview

This guide provides detailed instruction on training a traffic light detection model using the **[awml_tld](link)** repository and deploying it by converting to onnx model. If you wish to create a custom traffic light detection model with your own dataset, please follow the steps below. 

### Data preparation

#### Use Sample Dataset

Autoware  offers the sample dataset that illustrates the training procedures for traffic light detection. The dataset consists of 1062 images of traffic lights. All images are randomly cropped to imitate output of traffic_light_map_based_detector package. The dataset presented in VOC format. The  dataset is splited in two parts trainval and test. To use the sample dataset, please, download it from **[link](link to autoware aws)** and extract it to a designated folder of your choice. By default training configuration expects data in `awml_tld/TLDD` folder. 

#### Use Your Custom Dataset

To train the traffic light detection model you need to provide your data in Pascal VOC format. 

```shell
data_prefix/
│── Annotations/... annotation information with xml.
│   │── img_id1.xml
│   │── img_id2.xml
│   └── ...
│── JPEGImages/ ... raw image data.
│   │── img_id1.jpg
│   │── img_id2.jpg
│   └── ...
└── ImageSets/... annotation files to split train/test/val(e.g. train.txt)
    │── ann_file.txt
    └── ...   
```

The annotation format is shown as below.

```xml
<annotation>
<folder>xxxxyyyy</folder>
<filename>yyyzzz.jpg</filename>
<source>
    <database>Unknown</database>
    <annotation>Unknown</annotation>
    <image>Unknown</image>
</source>
<size>
    <width>xxx</width>
    <height>yyy</height>
    <depth>3</depth>
</size>
<segmented>0</segmented>
# annotations for each object
<object>
    <name>traffic_light</name>
    <occluded>0</occluded>
    <bndbox>
    <xmin>631.173828125</xmin>
    <ymin>332.12109375</ymin>
    <xmax>842.623046875</xmax>
    <ymax>415.24609375</ymax>
    </bndbox>
</object>
...
</annotation>
```

### Installation

#### Prerequisites

You need to have `poetry` installed on you system or in venv. You can follow officiall installation **[guide](https://python-poetry.org/docs/#installing-with-pipx)**.


#### Install awml_tld

```shell
$ git clone https://github.com/lexavtanke/Traffic_light_detector_training.git
$ cd awml_tld
$ poetry install 
```

#### Activate env

```shell
$ poetry shell
```

### Training

MMdetection provide a training script which is controlled by config file.
We provide training configuration for yolox_s model as example. Feel free to use it as parent for you configuration, so you will need to modify only desired part of the pipeline. 

```shell
# train detection ([] is optional)
$ mim train mmdet config/model_config/yolox/yolox_s_tld_416x416.py [--work_dir WORK_DIR --gpus NUM_GPUS --resume_from CHECKPOINT.pth]
```

### Export  YOLOX to ONNX model 

Before running conversion script be sure that you installed *onnx-graphsurgeon* with command:

```shell
$ pip install nvidia-pyindex
$ pip install onnx-graphsurgeon
```

To convert yolox model to onnx run:

```shell
# refer to yolox2onnx.py for details
$ python awml_tld/tools/yolox2onnx.py <MODEL.pth> --input_size 416 416 --model yolox-s
```


## Assumptions / Known limits

## Reference repositories

YOLOX github repository

- <https://github.com/Megvii-BaseDetection/YOLOX>
