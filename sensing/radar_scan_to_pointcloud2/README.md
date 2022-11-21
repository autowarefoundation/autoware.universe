# radar_scan_to_pointcloud2
## radar_scan_to_pointcloud2_node

- Convert from `radar_msgs::msg::RadarScan` to `sensor_msgs::msg::PointCloud2`
- Calculation cost O(n)
  - n: The number of radar return

### Input topics

| Name        | Type                       | Description |
| ----------- | -------------------------- | ----------- |
| input/radar | radar_msgs::msg::RadarScan | RadarScan   |

### Output topics

| Name         | Type                          | Description                  |
| ------------ | ----------------------------- | ---------------------------- |
| output/radar | sensor_msgs::msg::PointCloud2 | PointCloud2 radar pointcloud |

### Parameters

| Name                 | Type        | Description                                                                                                        |
| -------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------ |
| intensity_value_mode | std::string | The output's intensity value of pointcloud. Select from "amplitude" or "doppler_velocity". Default is "amplitude". |

### How to launch

```
ros2 launch radar_scan_to_pointcloud2 radar_scan_to_pointcloud2.launch.xml
```
