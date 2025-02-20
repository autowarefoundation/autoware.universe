# autoware_image_projection_based_fusion

## Purpose

The `autoware_image_projection_based_fusion` package is designed to enhance obstacle detection accuracy by integrating information from both image-based and LiDAR-based perception. It fuses detected obstacles—such as bounding boxes or segmentations—from 2D images with 3D point clouds or other obstacle representations, including bounding boxes, clusters, or segmentations. This fusion helps to refine obstacle classification and detection in autonomous driving applications.

### Fusion algorithms

The package provides multiple fusion algorithms, each designed for specific use cases. Below are the different fusion methods along with their descriptions and detailed documentation links:

| Fusion Name                | Description                                                                                                                                                                                              | Detail                                       |
| -------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| roi_cluster_fusion         | Assigns classification labels to LiDAR-detected clusters by matching them with Regions of Interest (ROIs) from a 2D object detector.                                                                     | [link](./docs/roi-cluster-fusion.md)         |
| roi_detected_object_fusion | Updates classification labels of detected objects using ROI information from a 2D object detector.                                                                                                       | [link](./docs/roi-detected-object-fusion.md) |
| pointpainting_fusion       | Augments the point cloud by painting each point with additional information from ROIs of a 2D object detector. The enriched point cloud is then processed by a 3D object detector for improved accuracy. | [link](./docs/pointpainting-fusion.md)       |
| roi_pointcloud_fusion      | Matching pointcloud with ROIs from a 2D object detector to detect unknown-labeled objects                                                                                                                | [link](./docs/roi-pointcloud-fusion.md)      |

## Inner Workings / Algorithms

![fusion_algorithm](./docs/images/fusion_algorithm.drawio.svg)

### Step 1: Matching and Creating a Collector

When a point cloud or a set of Regions of Interest (RoIs) arrives, its timestamp is checked, and an offset is subtracted to determine the reference timestamp. The node then searches for an existing collector with the same reference timestamp.

- If a matching collector is found, the incoming data is added to it.
- If no matching collector exists, a new collector is created and initialized with the reference timestamp.

### Step 2: Triggering the Timer

Once a collector is created, a countdown timer is started. The timeout duration depends on which message type arrived first and is defined by either `msg3d_timeout_sec` (for point clouds / clusters) or `rois_timeout_sec` (for RoIs).

The collector will attempt to fuse the collected 3D and 2D data either:

- When both point cloud and RoI data are available, or
- When the timer expires.

If no point cloud data is received before the timer expires, the collector will discard the data without performing fusion.

### Step 3: Fusion Process

The fusion process consists of three main stages:

1. **Preprocessing** – Preparing the input data for fusion.
2. **Fusion** – Aligning and merging RoIs with the 3D point cloud.
3. **Postprocessing** – Refining the fused output based on the algorithm's requirements.

The specific operations performed during these stages may vary depending on the type of fusion being applied.

### Step 4: Publishing the Fused Result

After the fusion process is completed, the fused output is published. The collector is then reset to an idle state, ready to process the next incoming message.

The figure below shows how the input data is fused in different scenarios.
![roi_sync_image2](./docs/images/roi_sync_2.png)

## Parameters

### Fusion Node Common Parameters

All of the fusion nodes have the common parameters described in the following

{{ json_to_markdown("perception/autoware_image_projection_based_fusion/schema/fusion_common.schema.json") }}

### Parameter Settings

The order in which `RoIs` or the `msg3d` message arrives at the fusion node depends on your system and sensor configuration. Since the primary goal is to fuse `2D RoIs` with `msg3d` data, `msg3d` is essential for processing.

If `RoIs` arrive earlier, they must wait until `msg3d` is received. You can adjust the waiting time using the `rois_timeout_sec` parameter.

If `msg3d` arrives first, the fusion process should proceed as quickly as possible, so the waiting time for `msg3d` (`msg3d_timeout_sec`) should be kept minimal.

#### Sensor Offset Considerations

The offset between each camera and the LiDAR is determined by their shutter timing. Users must understand the timing offset between the RoIs and msg3d to ensure accurate fusion. Once you know the offset between the RoIs and msg3d, fill the offsets in the parameter `rois_timestamp_offsets`.

#### Matching Strategies

We provide two matching strategies for different scenarios:

##### Naive Mode

User should use this mode if the concatenation node from the Autoware point cloud preprocessor is not being used. User should also set an appropriate `threshold` value to determine whether the time interval between the `msg3d` and `RoI` messages is within an acceptable range.
If the interval is less than the match threshold, the messages are considered matched.
![roi_sync_image1](./docs/images/roi_sync_1.png)

##### Advanced Mode

If the concatenation node from the Autoware point cloud preprocessor is being used, enable this mode.
The advanced mode parses diagnostics from the concatenation node to verify whether all point clouds have been successfully concatenated. If concatenation is incomplete, it dynamically adjusts `rois_timestamp_offsets` based on diagnostic messages.
Instead of using a fixed threshold, this mode requires setting two parameters:

- `msg3d_noise_window` (a single value)
- `rois_timestamp_noise_window` (a vector)

These parameters enforce stricter matching between the `RoI` messages and `msg3d` input.

#### Approximate camera projection

For algorithms like `pointpainting_fusion`, the computation required to project points onto an unrectified (raw) image can be substantial. To address this, an option is provided to reduce the computational load. Set the [`approximate_camera_projection parameter`](config/fusion_common.param.yaml) to `true` for each camera (ROIs). If the corresponding `point_project_to_unrectified_image` parameter is also set to true, the projections will be pre-cached.

The cached projections are calculated using a grid, with the grid size specified by the `approximation_grid_width_size` and `approximation_grid_height_size` parameters in the [configuration file](config/fusion_common.param.yaml). The centers of the grid are used as the projected points.

## Debug and Diagnostics

To verify whether the node has successfully fuse the msg3d or rois, the user can examine rqt or the `/diagnostics` topic using the following command:

```bash
ros2 topic echo /diagnostics
```

Below is an example output when the fusion is success:

- msg3d has a value of `True`.
- Each rois has a value of `True`.
- The `fusion_success` is `True`.
- The `level` value is `\0`. (diagnostic_msgs::msg::DiagnosticStatus::OK)

```bash
header:
  stamp:
    sec: 1722492015
    nanosec: 848508777
  frame_id: ''
status:
- level: "\0"
  name: 'pointpainting: pointpainting_fusion_status'
  message: Fused output is published and include all rois and msg3d
  hardware_id: pointpainting_fusion_checker
  values:
  - key: msg3d/is_fused
    value: 'True'
  - key: fused_timestamp
    value: '1738725170.860273600'
  - key: reference_timestamp_min
    value: '1738725170.850771904'
  - key: reference_timestamp_max
    value: '1738725170.870771885'
  - key: /rois0/timestamp
    value: '1738725170.903310537'
  - key: /rois0/is_fused
    value: 'True'
  - key: /rois1/timestamp
    value: '1738725170.934378386'
  - key: /rois1/is_fused
    value: 'True'
  - key: /rois2/timestamp
    value: '1738725170.917550087'
  - key: /rois2/is_fused
    value: 'True'
  - key: fusion_success
    value: 'True'
```

Below is an example output when the fusion is failed:

- msg3d has a value of `True`.
- Each rois has a value of `False`.
- The `fusion_success` is `False`.
- The `level` value is `\x02`. (diagnostic_msgs::msg::DiagnosticStatus::ERROR)

```bash
header:
  stamp:
    sec: 1722492015
    nanosec: 848508777
  frame_id: ''
status:
- level: "\x02"
  name: 'pointpainting: pointpainting_fusion_status'
  message: Fused output msg is published but misses some ROIs
  hardware_id: pointpainting_fusion_checker
  values:
  - key: msg3d/is_fused
    value: 'True'
  - key: fused_timestamp
    value: '1738725170.860273600'
  - key: reference_timestamp_min
    value: '1738725170.850771904'
  - key: reference_timestamp_max
    value: '1738725170.870771885'
  - key: /rois0/is_fused
    value: 'False'
  - key: /rois1/timestamp
    value: '1738725170.934378386'
  - key: /rois1/is_fused
    value: 'True'
  - key: /rois2/timestamp
    value: '1738725170.917550087'
  - key: /rois2/is_fused
    value: 'True'
  - key: fusion_success
    value: 'False'
```

## The `build_only` option

The `pointpainting_fusion` node has `build_only` option to build the TensorRT engine file from the ONNX file.
Although it is preferred to move all the ROS parameters in `.param.yaml` file in Autoware Universe, the `build_only` option is not moved to the `.param.yaml` file for now, because it may be used as a flag to execute the build as a pre-task. You can execute with the following command:

```bash
ros2 launch autoware_image_projection_based_fusion pointpainting_fusion.launch.xml model_name:=pointpainting model_path:=/home/autoware/autoware_data/image_projection_based_fusion model_param_path:=$(ros2 pkg prefix autoware_image_projection_based_fusion --share)/config/pointpainting.param.yaml build_only:=true
```
