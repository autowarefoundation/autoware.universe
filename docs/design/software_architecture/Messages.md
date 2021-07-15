# Messages

## Overview

This page describes the eight categories of message in the new architecture, along with definitions for each message.

- [Autoware control messages](#autoware-control-messages)
- [Autoware lanelet2 messages](#autoware-lanelet2-messages)
- [Autoware perception messages](#autoware-perception-messages)
- [Autoware planning messages](#autoware-planning-messages)
- [Autoware system messages](#autoware-system-messages)
- [Autoware traffic light messages](#autoware-traffic-light-messages)
- [Autoware vector map messages](#autoware-vector-map-messages)
- [Autoware vehicle messages](#autoware-vehicle-messages)

## Autoware control messages

### ControlCommand.msg

`float64 steering_angle`  
`float64 steering_angle_velocity`  
`float64 velocity`  
`float64 acceleration`

### ControlCommandStamped.msg

`Header header`  
`autoware_control_msgs/ControlCommand control`

## Autoware lanelet2 messages

### MapBin.msg

`Header header`  
`string format_version`  
`string map_version`  
`int8[] data`

## Autoware perception messages

### DynamicObject.msg

`uuid_msgs/UniqueID id`  
`Semantic semantic`  
`State state`  
`Shape shape`

### DynamicObjectArray.msg

`std_msgs/Header header`  
`DynamicObject[] objects`

### DynamicObjectWithFeature.msg

`DynamicObject object`  
`Feature feature`

### DynamicObjectWithFeatureArray.msg

`std_msgs/Header header`  
`DynamicObjectWithFeature[] feature_objects`

### Feature.msg

`sensor_msgs/PointCloud2 cluster`  
`sensor_msgs/RegionOfInterest roi`

### PredictedPath.msg

`geometry_msgs/PoseWithCovarianceStamped[] path`  
`float64 confidence`

### Semantic.msg

`uint8 UNKNOWN=0`  
`uint8 CAR=1`  
`uint8 TRUCK=2`  
`uint8 BUS=3`  
`uint8 BICYCLE=4`  
`uint8 MOTORBIKE=5`  
`uint8 PEDESTRIAN=6`  
`uint8 ANIMAL=7`  
`uint32 type`  
`float64 confidence`

### Shape.msg

`uint8 BOUNDING_BOX=0`  
`uint8 CYLINDER=1`  
`uint8 POLYGON=2`  
`uint8 type`  
`geometry_msgs/Vector3 dimensions`  
`geometry_msgs/Polygon footprint`

### State.msg

`geometry_msgs/PoseWithCovariance pose_covariance`  
`bool orientation_reliable`  
`geometry_msgs/TwistWithCovariance twist_covariance`  
`bool twist_reliable`  
`geometry_msgs/AccelWithCovariance acceleration_covariance`  
`bool acceleration_reliable`  
`PredictedPath[] predicted_paths`

## Autoware planning messages

### LaneSequence.msg

`int64[] lane_ids`

### Path.msg

`std_msgs/Header header`  
`autoware_planning_msgs/PathPoint[] points`  
`nav_msgs/OccupancyGrid drivable_area`

### PathPoint.msg

`uint8 REFERENCE=0`  
`uint8 FIXED=1`  
`geometry_msgs/Pose pose`  
`geometry_msgs/Twist twist`  
`uint8 type`

### PathPointWithLaneId.msg

`autoware_planning_msgs/PathPoint point`  
`int64[] lane_ids`

### PathWithLaneId.msg

`std_msgs/Header header`  
`autoware_planning_msgs/PathPointWithLaneId[] points`  
`nav_msgs/OccupancyGrid drivable_area`

### Route.msg

`std_msgs/Header header`  
`geometry_msgs/Pose goal_pose`  
`autoware_planning_msgs/RouteSection[] route_sections`

### RouteSection.msg

`int64[] lane_ids`  
`int64 preferred_lane_id`  
`int64[] continued_lane_ids`

### Scenario.msg

`string Empty=Empty`  
`string LaneDriving=LaneDriving`  
`string Parking=Parking`  
`string current_scenario`  
`string[] activating_scenarios`

### Trajectory.msg

`std_msgs/Header header`  
`autoware_planning_msgs/TrajectoryPoint[] points`

### TrajectoryPoint.msg

`geometry_msgs/Pose pose`  
`geometry_msgs/Twist twist`  
`geometry_msgs/Accel accel`

## Autoware system messages

### AutowareState.msg

`string Error=Error`  
`string InitializingVehicle=InitializingVehicle`  
`string WaitingForRoute=WaitingForRoute`  
`string Planning=Planning`  
`string WaitingForEngage=WaitingForEngage`  
`string Driving=Driving`  
`string ArrivedGoal=ArrivedGoal`  
`string FailedToArriveGoal=FailedToArriveGoal`  
`string state`  
`string msg`

## Autoware traffic light messages

### LampState.msg

`uint8 UNKNOWN=0`  
`uint8 RED=1`  
`uint8 GREEN=2`  
`uint8 YELLOW=3`  
`uint8 LEFT=4`  
`uint8 RIGHT=5`  
`uint8 UP=6`  
`uint8 DOWN=7`  
`uint32 type`  
`float32 confidence`

### TrafficLightRoi.msg

`sensor_msgs/RegionOfInterest roi`  
`int32 id`

### TrafficLightRoiArray.msg

`std_msgs/Header header`  
`autoware_traffic_light_msgs/TrafficLightRoi[] rois`

### TrafficLightState.msg

`autoware_traffic_light_msgs/LampState[] lamp_states`  
`int32 id`

### TrafficLightStateArray.msg

`std_msgs/Header header`  
`autoware_traffic_light_msgs/TrafficLightState[] states`

## Autoware vector map messages

### BinaryGpkgMap.msg

`std_msgs/Header header`  
`string format_version`  
`string map_version`  
`int8[] data`

## Autoware vehicle messages

### ControlMode.msg

`std_msgs/Header header`  
`uint8 MANUAL = 0`  
`uint8 AUTO = 1`  
`uint8 AUTO_STEER_ONLY = 2`  
`uint8 AUTO_PEDAL_ONLY = 3`  
`int32 data`

### Pedal.msg

`std_msgs/Header header`  
`float64 throttle`  
`float64 brake`

### Shift.msg

`uint8 NONE=0`  
`uint8 PARKING=1`  
`uint8 REVERSE=2`  
`uint8 NEUTRAL=3`  
`uint8 DRIVE=4`  
`uint8 LOW=5`  
`int32 data`

### ShiftStamped.msg

`std_msgs/Header header`  
`autoware_vehicle_msgs/Shift shift`

### Steering.msg

`std_msgs/Header header`  
`float32 data`

### TurnSignal.msg

`std_msgs/Header header`  
`uint8 NONE = 0`  
`uint8 LEFT = 1`  
`uint8 RIGHT = 2`  
`uint8 HAZARD = 3`  
`int32 data`

### VehicleCommand.msg

`std_msgs/Header header`  
`autoware_control_msgs/ControlCommand control`  
`autoware_vehicle_msgs/Shift shift`  
`int32 emergency`
