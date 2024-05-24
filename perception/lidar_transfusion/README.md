# lidar_transfusion

## Purpose

The `lidar_transfusion` package is used for 3D object detection based on lidar data (x, y, z, intensity).

## Inner-workings / Algorithms

The implementation bases on TransFusion [1] work. It uses TensorRT library for data process and network inference.

We trained the models using <https://github.com/open-mmlab/mmdetection3d>.

## Inputs / Outputs

### Input

| Name                 | Type                            | Description       |
| -------------------- | ------------------------------- | ----------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | Input pointcloud. |

### Output

| Name                                   | Type                                                  | Description                 |
| -------------------------------------- | ----------------------------------------------------- | --------------------------- |
| `~/output/objects`                     | `autoware_auto_perception_msgs::msg::DetectedObjects` | Detected objects.           |
| `debug/cyclic_time_ms`                 | `tier4_debug_msgs::msg::Float64Stamped`               | Cyclic time (ms).           |
| `debug/pipeline_latency_ms`            | `tier4_debug_msgs::msg::Float64Stamped`               | Pipeline latency time (ms). |
| `debug/processing_time/preprocess_ms`  | `tier4_debug_msgs::msg::Float64Stamped`               | Preprocess (ms).            |
| `debug/processing_time/inference_ms`   | `tier4_debug_msgs::msg::Float64Stamped`               | Inference time (ms).        |
| `debug/processing_time/postprocess_ms` | `tier4_debug_msgs::msg::Float64Stamped`               | Postprocess time (ms).      |
| `debug/processing_time/total_ms`       | `tier4_debug_msgs::msg::Float64Stamped`               | Total processing time (ms). |

## Parameters

### Core Parameters

| Name                             | Type         | Default Value | Description                                                                                        |
| -------------------------------- | ------------ | ------------- | -------------------------------------------------------------------------------------------------- |
| `class_names`                    | list[string] | -             | Class names for 3D object detection.                                                               |
| `trt_precision`                  | string       | `fp16`        | TensorRT inference precision: `fp32` or `fp16`.                                                    |
| `voxels_num`                     | list[int]    | -             | Voxels input dimension [min, opt, max]. It propagates to points and coordinates dimension as well. |
| `pointcloud_range`               | list[double] | -             | Pointcloud range to process [-x, -y, -z, x, y, z].                                                 |
| `voxel_size`                     | list[double] | -             | Voxel size [x, y, z].                                                                              |
| `onnx_path`                      | string       | `""`          | Path to ONNX file.                                                                                 |
| `engine_path`                    | string       | `""`          | Path to TensorRT Engine file.                                                                      |
| `densification_num_past_frames`  | int          | `1`           | The number of past frames to fuse with the current frame.                                          |
| `densification_world_frame_id`   | string       | `map`         | The world frame id to fuse multi-frame pointcloud.                                                 |
| `circle_nms_dist_threshold`      | float        | `0.5`         | Distance threshold for circle-based Non Maximum Suppression.                                       |
| `iou_nms_target_class_names`     | list[string] | -             | Target classes for IoU-based Non Maximum Suppression.                                              |
| `iou_nms_search_distance_2d`     | double       | -             | If two objects are farther than the value, NMS isn't applied.                                      |
| `iou_nms_threshold`              | double       | -             | IoU threshold for the IoU-based Non Maximum Suppression.                                           |
| `yaw_norm_threshold`             | list[double] | -             | Yaw angle normalization thresholds. Disparity below threshold suppress class confidence to 0.0.    |
| `score_threshold`                | float        | `0.2`         | Detected objects with score less than threshold are ignored.                                       |
| `allow_remapping_by_area_matrix` | list[bool]   | -             | Whether remapping by area matrix is allowed.                                                       |
| `min_area_matrix`                | list[double] | -             | Minimum area values for remapping by area matrix.                                                  |
| `max_area_matrix`                | list[double] | -             | Maximum area values for remapping by area matrix.                                                  |

### The `build_only` option

The `lidar_transfusion` node has `build_only` option to build the TensorRT engine file from the ONNX file.
Although it is preferred to move all the ROS parameters in `.param.yaml` file in Autoware Universe, the `build_only` option is not moved to the `.param.yaml` file for now, because it may be used as a flag to execute the build as a pre-task. You can execute with the following command:

```bash
ros2 launch lidar_transfusion lidar_transfusion.launch.xml build_only:=true
```

### The `log_level` option

The default logging severity level for `lidar_transfusion` is `info`. For debugging purposes, the developer may decrease severity level using `log_level` parameter:

```bash
ros2 launch lidar_transfusion lidar_transfusion.launch.xml log_level:=debug
```

### The `HOST_PROCESSING` build definition

<!-- cSpell:ignore DHOST_PROCESSING -->

You can force host (CPU) point cloud processing with `-DHOST_PROCESSING` flag while using `colcon build` syntax. Note that HOST processing only supports `FLOAT32` cloud points.

## Assumptions / Known limits

## Trained Models

You can download the onnx format of trained models by clicking on the links below.

- Transfusion: [transfusion.onnx](https://awf.ml.dev.web.auto/perception/models/transfusion/v1/transfusion.onnx)

The model was trained in TIER IV's internal database (~11k lidar frames) for 20 epochs.

### Changelog

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## References/External links

[1] Xuyang Bai, Zeyu Hu, Xinge Zhu, Qingqiu Huang, Yilun Chen, Hongbo Fu and Chiew-Lan Tai. "TransFusion: Robust LiDAR-Camera Fusion for 3D Object Detection with Transformers." arXiv preprint arXiv:2203.11496 (2022). <!-- cspell:disable-line -->

[2] <https://github.com/wep21/CUDA-TransFusion>

[3] <https://github.com/open-mmlab/mmdetection3d>

[4] <https://github.com/open-mmlab/OpenPCDet>

[5] <https://www.nuscenes.org/nuscenes>

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
