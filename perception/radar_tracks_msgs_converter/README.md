# radar_tracks_msgs_converter

This package convert from [radar_msgs/msg/RadarTracks](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) into [autoware_auto_perception_msgs/msg/TrackedObject](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/TrackedObject.idl).

- Calculation cost is O(n).
  - n: The number of radar objects

## Design
### Input / Output

- Input
  - `~/input/radar_objects` (radar_msgs/msg/RadarTracks.msg): Converted topic
  - `~/input/twist` (geometry_msgs/msg/TwistStamped.msg): Ego vehicle twist
- Output
  - `~/output/radar_objects` (autoware_auto_perception_msgs/msg/TrackedObject.msg): Converted topic

### Parameters

- `update_rate_hz` (double): The update rate [hz].
  - Default parameter is 20.0
- `new_frame_id` (string): The header frame of output topic.
  - Default parameter is "base_link"
- `use_twist_compensation` (bool): If the parameter is true, then the twist of output objects' topic is compensated by ego vehicle motion.
  - Default parameter is "false"
