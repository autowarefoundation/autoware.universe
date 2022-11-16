# radar_threshold_filter

## radar_threshold_filter_node

- Remove noise from radar pointcloud
  - amplitude filter
    - Low amplitude consider noise
  - FOV filter
    - Pointcloud from radar's FOV edge occur perturbation
  - Range filter
    - Too near pointcloud often occur noise
- Calculation cost O(n)
  - n: the number of radar pointcloud

### Input topics

| Name        | Type                         | Description           |
| ----------- | ---------------------------- | --------------------- |
| input/radar | radar_msgs/msg/RadarScan.msg | Radar pointcloud data |

### Output topics

| Name         | Type                         | Description               |
| ------------ | ---------------------------- | ------------------------- |
| output/radar | radar_msgs/msg/RadarScan.msg | Filtered radar pointcloud |

### Parameters

- For node parameter

| Name                    | Type   | Description                                                                                           |
| ----------------------- | ------ | ----------------------------------------------------------------------------------------------------- |
| update_rate             | double | node Hz                                                                                               |
| is_amplitude_filter     | bool   | if this parameter is true, apply amplitude filter (publish amplitude_min < amplitude < amplitude_max) |
| amplitude_min           | double | [dBm^2]                                                                                               |
| amplitude_max           | double | [dBm^2]                                                                                               |
| is_range_filter         | bool   | if this parameter is true, apply range filter (publish range_min < range < range_max)                 |
| range_min               | double | [m]                                                                                                   |
| range_max               | double | [m]                                                                                                   |
| is_angle_azimuth_filter | bool   | if this parameter is true, apply angle filter (publish angle_azimuth_min < range < angle_azimuth_max) |
| angle_azimuth_min       | double | [rad]                                                                                                 |
| angle_azimuth_max       | double | [rad]                                                                                                 |
| is_z_filter             | bool   | if this parameter is true, apply z position filter (publish z_min < z < z_max)                        |
| z_min                   | double | [m]                                                                                                   |
| z_max                   | double | [m]                                                                                                   |

### How to launch

```sh
ros2 launch radar_threshold_filter radar_threshold_filter.launch.xml
```
