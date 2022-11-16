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

| Name                       | Type   | Description                                                                                               |
| -------------------------- | ------ | --------------------------------------------------------------------------------------------------------- |
| update\_rate               | double | node Hz                                                                                                   |
| is\_amplitude\_filter      | bool   | if this parameter is true, apply amplitude filter (publish amplitude\_min < amplitude < amplitude\_max)   |
| amplitude\_min             | double | [dBm^2]                                                                                                   |
| amplitude\_max             | double | [dBm^2]                                                                                                   |
| is\_range\_filter          | bool   | if this parameter is true, apply range filter (publish range\_min < range < range\_max)                   |
| range\_min                 | double | [m]                                                                                                       |
| range\_max                 | double | [m]                                                                                                       |
| is\_angle\_azimuth\_filter | bool   | if this parameter is true, apply angle filter (publish angle\_azimuth\_min < range < angle\_azimuth\_max) |
| angle\_azimuth\_min        | double | [rad]                                                                                                     |
| angle\_azimuth\_max        | double | [rad]                                                                                                     |
| is\_z\_filter              | bool   | if this parameter is true, apply z position filter (publish z\_min < z < z\_max)                          |
| z\_min                     | double | [m]                                                                                                       |
| z\_max                     | double | [m]                                                                                                       |

### How to launch

```sh
ros2 launch radar_threshold_filter radar_threshold_filter.launch.xml
```
