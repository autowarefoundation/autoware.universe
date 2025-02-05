# autoware_image_projection_based_fusion

## Purpose

The `autoware_image_projection_based_fusion` is a package to fuse detected obstacles (bounding box or segmentation) from image and 3d pointcloud or obstacles (bounding box, cluster or segmentation).

## Inner-workings / Algorithms

### Sync Algorithm

#### matching

The offset between each camera and the lidar is set according to their shutter timing.
After applying the offset to the timestamp, if the interval between the timestamp of pointcloud topic and the roi message is less than the match threshold, the two messages are matched.

![roi_sync_image1](./docs/images/roi_sync_1.png)

current default value at autoware.universe for TIER IV Robotaxi are: - input_offset_ms: [61.67, 111.67, 45.0, 28.33, 78.33, 95.0] - match_threshold_ms: 30.0

#### fusion and timer

![roi_sync_image2](./docs/images/roi_sync_2.png)

The subscription status of the message is signed with 'O'.

1.if a pointcloud message is subscribed under the below condition:

|                     | pointcloud | roi msg 1 | roi msg 2 | roi msg 3 |
| :-----------------: | :--------: | :-------: | :-------: | :-------: |
| subscription status |            |     O     |     O     |     O     |

If the roi msgs can be matched, fuse them and postprocess the pointcloud message.
Otherwise, fuse the matched roi msgs and cache the pointcloud.

2.if a pointcloud message is subscribed under the below condition:

|                     | pointcloud | roi msg 1 | roi msg 2 | roi msg 3 |
| :-----------------: | :--------: | :-------: | :-------: | :-------: |
| subscription status |            |     O     |     O     |           |

if the roi msgs can be matched, fuse them and cache the pointcloud.

3.if a pointcloud message is subscribed under the below condition:

|                     | pointcloud | roi msg 1 | roi msg 2 | roi msg 3 |
| :-----------------: | :--------: | :-------: | :-------: | :-------: |
| subscription status |     O      |     O     |     O     |           |

If the roi msg 3 is subscribed before the next pointcloud message coming or timeout, fuse it if matched, otherwise wait for the next roi msg 3.
If the roi msg 3 is not subscribed before the next pointcloud message coming or timeout, postprocess the pointcloud message as it is.

The timeout threshold should be set according to the postprocessing time.
E.g, if the postprocessing time is around 50ms, the timeout threshold should be set smaller than 50ms, so that the whole processing time could be less than 100ms.
current default value at autoware.universe for XX1: - timeout_ms: 50.0

#### The `build_only` option

The `pointpainting_fusion` node has `build_only` option to build the TensorRT engine file from the ONNX file.
Although it is preferred to move all the ROS parameters in `.param.yaml` file in Autoware Universe, the `build_only` option is not moved to the `.param.yaml` file for now, because it may be used as a flag to execute the build as a pre-task. You can execute with the following command:

```bash
ros2 launch autoware_image_projection_based_fusion pointpainting_fusion.launch.xml model_name:=pointpainting model_path:=/home/autoware/autoware_data/image_projection_based_fusion model_param_path:=$(ros2 pkg prefix autoware_image_projection_based_fusion --share)/config/pointpainting.param.yaml build_only:=true
```

#### Known Limits

The rclcpp::TimerBase timer could not break a for loop, therefore even if time is out when fusing a roi msg at the middle, the program will run until all msgs are fused.

### Approximate camera projection

For algorithms like `pointpainting_fusion`, the computation required to project points onto an unrectified (raw) image can be substantial. To address this, an option is provided to reduce the computational load. Set the [`approximate_camera_projection parameter`](config/fusion_common.param.yaml) to `true` for each camera (ROIs). If the corresponding `point_project_to_unrectified_image` parameter is also set to true, the projections will be pre-cached.

The cached projections are calculated using a grid, with the grid size specified by the `approximation_grid_width_size` and `approximation_grid_height_size` parameters in the [configuration file](config/fusion_common.param.yaml). The centers of the grid are used as the projected points.

### Detail description of each fusion's algorithm is in the following links

| Fusion Name                | Description                                                                                     | Detail                                       |
| -------------------------- | ----------------------------------------------------------------------------------------------- | -------------------------------------------- |
| roi_cluster_fusion         | Overwrite a classification label of clusters by that of ROIs from a 2D object detector.         | [link](./docs/roi-cluster-fusion.md)         |
| roi_detected_object_fusion | Overwrite a classification label of detected objects by that of ROIs from a 2D object detector. | [link](./docs/roi-detected-object-fusion.md) |
| pointpainting_fusion       | Paint the point cloud with the ROIs from a 2D object detector and feed to a 3D object detector. | [link](./docs/pointpainting-fusion.md)       |
| roi_pointcloud_fusion      | Matching pointcloud with ROIs from a 2D object detector to detect unknown-labeled objects       | [link](./docs/roi-pointcloud-fusion.md)      |
