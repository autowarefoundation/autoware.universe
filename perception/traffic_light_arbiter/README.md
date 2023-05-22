# traffic_light_arbiter

## Purpose

This package receives traffic signals from perception and V2X components and combines them using either a confidence-based or a V2X-preference based approach.

## TrafficLightArbiter

A node that merges traffic light/signal state from image recognition and V2X to provide a planning component.
It's currently a provisional implementation.

### Inputs / Outputs

#### Input

| Name                             | Type                                         | Description                                              |
| -------------------------------- | -------------------------------------------- | -------------------------------------------------------- |
| ~/sub/vector_map                 | autoware_auto_mapping_msgs/msg/HADMapBin     | The vector map to get valid traffic signal ids.          |
| ~/sub/perception_traffic_signals | autoware_perception_msgs::msg::TrafficSignal | The traffic signals from the image recognition pipeline. |
| ~/sub/v2x_traffic_signals        | autoware_perception_msgs::msg::TrafficSignal | The traffic signals from the V2X interface.              |

#### Output

| Name                  | Type                                            | Description                      |
| --------------------- | ----------------------------------------------- | -------------------------------- |
| ~/pub/traffic_signals | autoware_perception_msgs/msg/TrafficSignalArray | The merged traffic signal state. |

## Parameters

### Core Parameters

| Name                        | Type   | Default Value | Description                                                                                                                |
| --------------------------- | ------ | ------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `v2x_time_tolerance`        | double | 5.0           | The duration in seconds a V2X message is considered valid for merging                                                      |
| `perception_time_tolerance` | double | 1.0           | The duration in seconds a perception message is considered valid for merging                                               |
| `v2x_priority`              | bool   | false         | Whether or not V2X signals take precedence over perception-based ones. If false, the merging uses confidence as a criteria |
