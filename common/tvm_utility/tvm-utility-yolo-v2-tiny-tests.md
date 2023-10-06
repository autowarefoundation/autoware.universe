# YOLOv2 Tiny Example Pipeline

This is an example implementation of an inference pipeline using the pipeline
framework. This example pipeline executes the
[YOLO V2 Tiny](https://pjreddie.com/darknet/yolov2/) model and decodes its
output.

## Compiling the Example

1. Check if model was downloaded during the env preparation step by ansible and
   models files exist in the folder $HOME/autoware_data/tvm_utility/models/yolo_v2_tiny.

   If not you can download them manually, see [Manual Artifacts Downloading](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts).

2. Download an example image to be used as test input. This image needs to be
   saved in the `artifacts/yolo_v2_tiny/` folder

```sh
curl https://raw.githubusercontent.com/pjreddie/darknet/master/data/dog.jpg \
  > artifacts/yolo_v2_tiny/test_image_0.jpg
```

1. Build and test.

```sh
colcon build --packages-up-to tvm_utility --cmake-args -DBUILD_EXAMPLE=ON
colcon test --packages-select tvm_utility
```

## GPU backend

Vulkan is supported by default by the tvm_vendor package.
It can be selected by setting the `tvm_utility_BACKEND` variable:

```sh
colcon build --packages-up-to tvm_utility -Dtvm_utility_BACKEND=vulkan
```
