Detection
=====
## Use Cases and Requirements
Prediction in Object Recognition is required for usecases involved with obstacles:
* Changing lane
* Turning at intersection
* Stopping at a crosswalk when pedestrians are walking
* Passing intersection without traffic lights
* Merging into another lane
* Taking over Pedestrian/Cyclists
* Stopping/yielding to an obstacle

For the details about related requirements, please refer to the [document for Perception stack](/design/Perception/Perception.md).

## Role
Prediction in Object Recognition estimate objects' intention. Intentions are represented as objects' future trajectories with covariance. The Planning module makes a decision and plans a future ego-motion based on the results of predicted objects.

## Input

| Input       | Data Type | Topic |
|-|-|-|
| Dynamic Objects       | `autoware_perception_msgs::DynamicObjectArray`|/perception/object_recognition/tracking/objects|
|Map|`autoware_lanelet2_msgs::MapBin`|/map/vector_map|
|TF  | `tf2_msgs::TFMessage`|/tf|

## Output

| Output       | Data Type| Output Componenet | TF Frame | Topic|
|----|-|-|-|-|
|Dynamic Objects|`autoware_perception_msgs::DynamicObjectArray`|Planning| `map`|/perception/object_recognition/objects|

## Design
This is our sample implementation for the Tracking module.
![msg](/design/img/ObjectPredictionDesign.png)


## Requirement in Output
Designated objects' property in autoware_perception_msgs::DynamicObject needs to be filled in the Prediction module before passing to the Planning component.

![msg](/design/img/ObjectPredictionRequirement.png)


| Property  | Definition |Data Type                                 | Parent Data Type|
|-------------|--|-------------------------------------------|----|
| predicted_path      | Predicted furuter paths for an object.|`autoware_perception_msgs::PredictedPath[]	`|`autoware_perception_msgs::State` |
