# concatenate_and_time_synchronize_node

## Purpose

The `concatenate_and_time_synchronize_node` is a ROS2 node that combines and synchronizes multiple point clouds into a single concatenated point cloud. This enhances the sensing range for autonomous driving vehicles by integrating data from multiple LiDARs.

## Inner Workings / Algorithms

![concatenate_algorithm](./image/concatenate_algorithm.drawio.svg)

### Step 1: Match and Create Collector

When a point cloud arrives, its timestamp is checked, and an offset is subtracted to get the reference timestamp. The node then checks if there is an existing collector with the same reference timestamp. If such a collector exists, the point cloud is added to it. If no such collector exists, a new collector is created with the reference timestamp.

### Step 2: Trigger the Timer

Once a collector is created, a timer for that collector starts counting down (this value is defined by `timeout_sec`). The collector begins to concatenate the point clouds either when the required number of point clouds has been collected or when the timer counts down to zero.

### Step 3: Concatenate the Point Clouds

The concatenation process involves merging multiple point clouds into a single, concatenated point cloud. The timestamp of the concatenated point cloud will be the earliest timestamp from the input point clouds. By setting the parameter `is_motion_compensated` to `true`, the node will consider the timestamps of the input point clouds and utilize the `twist` information from `geometry_msgs::msg::TwistWithCovarianceStamped` to compensate for motion, aligning the point cloud to the selected (earliest) timestamp.

### Step 4: Publish the Point Cloud

After concatenation, the concatenated point cloud is published, and the collector is deleted to free up resources.

## Inputs / Outputs

### Input

| Name            | Type                                             | Description                                                                       |
| --------------- | ------------------------------------------------ | --------------------------------------------------------------------------------- |
| `~/input/twist` | `geometry_msgs::msg::TwistWithCovarianceStamped` | The twist information used to interpolate the timestamp of each LiDAR point cloud |
| `~/input/odom`  | `nav_msgs::msg::Odometry`                        | The vehicle odometry used to interpolate the timestamp of each LiDAR point cloud  |

By setting the `input_twist_topic_type` parameter to `twist` or `odom`, the subscriber will subscribe to either `~/input/twist` or `~/input/odom`. If the user doesn't want to use the twist information or vehicle odometry to compensate for motion, set `is_motion_compensated` to `false`.

### Output

| Name              | Type                            | Description               |
| ----------------- | ------------------------------- | ------------------------- |
| `~/output/points` | `sensor_msgs::msg::Pointcloud2` | Concatenated point clouds |

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/concatenate_and_time_sync_node.schema.json") }}

### Parameter Settings

Three parameters, `timeout_sec`, `lidar_timestamp_offsets`, and `lidar_timestamp_noise_window`, are critical for collecting point clouds in the same collector and handling edge cases effectively.

#### timeout_sec

When network issues occur or when point clouds experience delays in the previous processing pipeline, some point clouds may be delayed or dropped. To address this, the `timeout_sec` parameter is used. If the timer reaches zero, the collector will not wait for delayed or dropped point clouds but will concatenate the remaining point clouds in the collector directly. The figure below demonstrates how `timeout_sec` works with `concatenate_and_time_sync_node`.

![concatenate_edge_case](./image/concatenate_edge_case.drawio.svg)

#### lidar_timestamp_offsets

Since different vehicles have varied designs for LiDAR scanning, the timestamps of each LiDAR may differ. Users need to know the offsets between each LiDAR and set the values in `lidar_timestamp_offsets`. For instance, if there are three LiDARs (left, right, top), and the timestamps for the left, right, and top point clouds are 0.01, 0.05, and 0.09 seconds respectively, the parameters should be set as [0.0, 0.04, 0.08]. This reflects the timestamp differences between the current point cloud and the point cloud with the earliest timestamp. Note that the order of the `lidar_timestamp_offsets` corresponds to the order of the `input_topics`.

The figure below demonstrates how `lidar_timestamp_offsets` works with `concatenate_and_time_sync_node`.

![ideal_timestamp_offset](./image/ideal_timestamp_offset.drawio.svg)

#### lidar_timestamp_noise_window

Additionally, due to the mechanical design of LiDARs, there may be some jitter in the timestamps of each scan. For example, if the scan frequency is set to 10 Hz (scanning every 100 ms), the timestamps between each scan might not be exactly 100 ms apart. To handle this noise, the `lidar_timestamp_noise_window` parameter is provided.

Take the left LiDAR from the above example: if the timestamps of the left point clouds are 0.01, 0.11, and 0.21 seconds, the timestamp is ideal without any noise. Then the example will be the same as above. However, if the timestamps of the left point clouds are 0.010, 0.115, and 0.210 seconds respectively, resulting in differences of 105 ms and 95 ms, the noise is 5 ms (compared to 100 ms). In this case, the user should set 0.005 in the `lidar_timestamp_noise_window` parameter.

The figure below demonstrates how `lidar_timestamp_noise_window` works with `concatenate_and_time_sync_node`. If the green `X` is in the range of the red triangles, it means that the point cloud matches the reference timestamp of the collector.

![noise_timestamp_offset](./image/noise_timestamp_offset.drawio.svg)

## Launch

```bash
# The launch file will read the parameters from the concatenate_and_time_sync_node.param.yaml
ros2 launch autoware_pointcloud_preprocessor concatenate_and_time_sync_node.launch.xml
```

## Test

```bash
# build autoware_pointcloud_preprocessor
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_pointcloud_preprocessor

# test autoware_pointcloud_preprocessor
colcon test --packages-select autoware_pointcloud_preprocessor --event-handlers console_cohesion+
```

## Debug and Diagnostics

To verify whether the node has successfully concatenated the point clouds, the user can examine the `/diagnostics` topic using the following command:

```bash
ros2 topic echo /diagnostics
```

Below is an example output when the point clouds are concatenated successfully:

- Each point cloud has a value of `1`.
- The `concatenate status` is `1`.
- The `level` value is `\0`. (diagnostic_msgs::msg::DiagnosticStatus::OK)

```bash
header:
  stamp:
    sec: 1722492015
    nanosec: 848508777
  frame_id: ''
status:
- level: "\0"
  name: 'concatenate_and_time_sync_node: concat_status'
  message: Concatenated pointcloud is published and include all topics
  hardware_id: concatenate_data_checker
  values:
  - key: concatenated cloud timestamp
    value: '1718260240.159229994'
  - key: reference timestamp min
    value: '1718260240.149230003'
  - key: reference timestamp max
    value: '1718260240.169229984'
  - key: /sensing/lidar/left/pointcloud_before_sync timestamp
    value: '1718260240.159229994'
  - key: /sensing/lidar/left/pointcloud_before_sync
    value: '1'
  - key: /sensing/lidar/right/pointcloud_before_sync timestamp
    value: '1718260240.194104910'
  - key: /sensing/lidar/right/pointcloud_before_sync
    value: '1'
  - key: /sensing/lidar/top/pointcloud_before_sync timestamp
    value: '1718260240.234578133'
  - key: /sensing/lidar/top/pointcloud_before_sync
    value: '1'
  - key: concatenate status
    value: '1'
```

Below is an example when point clouds fail to concatenate successfully.

- Some point clouds might have values of `0`.
- The `concatenate status` is `0`.
- The `level` value is `\x02`. (diagnostic_msgs::msg::DiagnosticStatus::ERROR)

```bash
header:
  stamp:
    sec: 1722492663
    nanosec: 344942959
  frame_id: ''
status:
- level: "\x02"
  name: 'concatenate_and_time_sync_node: concat_status'
  message: Concatenated pointcloud is published but miss some topics
  hardware_id: concatenate_data_checker
  values:
  - key: concatenated cloud timestamp
    value: '1718260240.859827995'
  - key: reference timestamp min
    value: '1718260240.849828005'
  - key: reference timestamp max
    value: '1718260240.869827986'
  - key: /sensing/lidar/left/pointcloud_before_sync timestamp
    value: '1718260240.859827995'
  - key: /sensing/lidar/left/pointcloud_before_sync
    value: '1'
  - key: /sensing/lidar/right/pointcloud_before_sync timestamp
    value: '1718260240.895193815'
  - key: /sensing/lidar/right/pointcloud_before_sync
    value: '1'
  - key: /sensing/lidar/top/pointcloud_before_sync
    value: '0'
  - key: concatenate status
    value: '0'
```

## Node separation options

There is also an option to separate the concatenate_and_time_sync_node into two nodes: one for `time synchronization` and another for `concatenate pointclouds` ([See this PR](https://github.com/autowarefoundation/autoware.universe/pull/3312)).

Note that the `concatenate_pointclouds` and `time_synchronizer_nodelet` are using the old design of the concatenate node.

## Assumptions / Known Limits

- If `is_motion_compensated` is set to `false`, the `concatenate_and_time_sync_node` will directly concatenate the point clouds without applying for motion compensation. This can save several milliseconds depending on the number of LiDARs being concatenated. Therefore, if the timestamp differences between point clouds are negligible, the user can set `is_motion_compensated` to `false` and omit the need for twist or odometry input for the node.
- As mentioned above, the user should clearly understand how their LiDAR's point cloud timestamps are managed to set the parameters correctly.
