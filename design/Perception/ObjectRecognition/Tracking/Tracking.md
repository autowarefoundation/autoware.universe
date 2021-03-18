Detection
=====

## Use Cases and Requirements
Tracking in Object Recognition is required for use cases involved with obstacles:
* Changing lane
* Turning at intersection
* Avoiding parked vehicles
* Stopping at a crosswalk when pedestrians are walking
* Passing intersection without traffic lights
* Merging into another lane
* Taking over Pedestrian/Cyclists
* Stopping/yielding to an obstacle

For the details about related requirements, please refer to the [document for Perception stack](/design/Perception/Perception.md).

## Role
Tracking in Object Recognition keeps objects' unique id over time. This time series processing leads to estimating objects' property such as their velocity and/or acceleration. Furthermore, it could estimate more accurate objects' orientation by leveraging the Detection results over time.

## Input

| Input       | Data Type|Topic
|-|-|-|
| Dynamic Objects       | `autoware_perception_msgs::DynamicObjectArray`|/perception/object_recognition/detection/objects|
|TF  | `tf2_msgs::TFMessage`|/tf|

## Output

| Output       | Data Type| Output Module | TF Frame| Topic|
|----|-|-|-|-|
|Dynamic Objects|`autoware_perception_msgs::DynamicObjectArray`|Object Recognition: Prediction| `map`|/perception/object_recognition/tracking/objects|

## Design
This is our sample implementation for the Tracking module.
![msg](/design/img/ObjectTrackingDesign.png)


## Requirement in Output

Designated objects' properties in autoware_perception_msgs::DynamicObject need to be filled in the Tracking module before passing to the Prediction module.

![msg](/design/img/ObjectTrackingRequirement.png)


| Property  | Definition |Data Type                                 | Parent Data Type|
|-------------|--|-------------------------------------------|----|
| id      | Unique object id over frames|`uuid_msgs::UniqueID`                 |`autoware_perception_msgs::DynamicObject`|
| twist        |Velocity in ROS twist format. |`geometry_msgs::TwistWithCovariance` |`autoware_perception_msgs::State`|
| twist_reliable |Boolean for stable twist or not.| `bool`           |`autoware_perception_msgs::State`|
| acceleration |Acceleration in ROS accel format.|`geometry_msgs::AccelWithCovariance`           |`autoware_perception_msgs::State`|
| acceleration_reliable |Boolean for stable acceleration or not.|`bool`           |`autoware_perception_msgs::State`|
