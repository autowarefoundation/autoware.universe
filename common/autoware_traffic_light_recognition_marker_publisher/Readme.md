# Traffic Light Recognition Marker Publisher

## Purpose

This node publishes a marker array for visualizing traffic signal recognition results on Rviz.

![sample_img](./images/traffic_light_recognition_visualization_sample.png)

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                                                    | Type                                                    | Description                                       |
| ------------------------------------------------------- | ------------------------------------------------------- | ------------------------------------------------- |
| `/map/vector_map`                                       | `autoware_map_msgs::msg::LaneletMapBin`                 | Vector map for getting traffic signal information |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | The result of traffic signal recognition          |

### Output

| Name                                                           | Type                                   | Description                                                                    |
| -------------------------------------------------------------- | -------------------------------------- | ------------------------------------------------------------------------------ |
| `/perception/traffic_light_recognition/traffic_signals_marker` | `visualization_msgs::msg::MarkerArray` | Publish a marker array for visualization of traffic signal recognition results |

## Parameters

None.

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

TBD.
