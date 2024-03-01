# radar_tracks_msgs_converter

This package converts from [radar_msgs/msg/RadarTracks](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) into [autoware_auto_perception_msgs/msg/DetectedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/DetectedObject.idl) and [autoware_auto_perception_msgs/msg/TrackedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/TrackedObject.idl).

- Calculation cost is O(n).
  - n: The number of radar objects

## Design

### Background

Autoware uses [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) as radar objects input data.
To use radar objects data for Autoware perception module easily, `radar_tracks_msgs_converter` converts message type from `radar_msgs/msg/RadarTracks.msg` to `autoware_auto_perception_msgs/msg/DetectedObject`.
In addition, because many detection module have an assumption on base_link frame, `radar_tracks_msgs_converter` provide the functions of transform frame_id.

### Note

`Radar_tracks_msgs_converter` converts the label from `radar_msgs/msg/RadarTrack.msg` to Autoware label.
Label id is defined as below.

|            | RadarTrack | Autoware |
| ---------- | ---------- | -------- |
| UNKNOWN    | 32000      | 0        |
| CAR        | 32001      | 1        |
| TRUCK      | 32002      | 2        |
| BUS        | 32003      | 3        |
| TRAILER    | 32004      | 4        |
| MOTORCYCLE | 32005      | 5        |
| BICYCLE    | 32006      | 6        |
| PEDESTRIAN | 32007      | 7        |

Additional vendor-specific classifications are permitted starting from 32000 in [radar_msgs/msg/RadarTrack.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg).
Autoware objects label is defined in [ObjectClassification.idl](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/ObjectClassification.idl)

## Interface

### Input

- `~/input/radar_objects` (`radar_msgs/msg/RadarTracks.msg`)
  - Input radar topic
- `~/input/odometry` (`nav_msgs/msg/Odometry.msg`)
  - Ego vehicle odometry topic

### Output

- `~/output/radar_detected_objects` (`autoware_auto_perception_msgs/msg/DetectedObject.idl`)
  - DetectedObject topic converted to Autoware message.
  - This is used for radar sensor fusion detection and radar detection.
- `~/output/radar_tracked_objects` (`autoware_auto_perception_msgs/msg/TrackedObject.idl`)
  - TrackedObject topic converted to Autoware message.
  - This is used for tracking layer sensor fusion.

### Parameters

{{ json_to_markdown("perception/radar_tracks_msgs_converter/schema/radar_tracks_msgs_converter.schema.json") }}
