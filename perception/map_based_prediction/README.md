# Map Based Prediction

## Role

`map_besed_prediction` is a module to predict the future paths of other vehicles and humans according to the shape of the map and the surrounding environment..

## Inputs / Outputs

### Input

| Name                                               | Type                                                 | Description                              |
| -------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------- |
| `~/perception/object_recognition/tracking/objects` | `autoware_auto_perception_msgs::msg::TrackedObjects` | tracking objects without predicted path. |
| `~/vector_map`                                     | `autoware_auto_mapping_msgs::msg::HADMapBin`         | binary data of Lanelet2 Map.             |

### Output

| Name                     | Type                                                   | Description                           |
| ------------------------ | ------------------------------------------------------ | ------------------------------------- |
| `~/objects`              | `autoware_auto_perception_msgs::msg::PredictedObjects` | tracking objects with predicted path. |
| `~/objects_path_markers` | `visualization_msgs::msg::MarkerArray`                 | marker for visualization.             |

## Parameters

| Parameter                        | Type   | Description                                |
| -------------------------------- | ------ | ------------------------------------------ |
| `has_subscribed_map`             | bool   | true when subscribed path exists           |
| `prediction_time_horizon`        | double | predict time duration for predicted path   |
| `prediction_sampling_delta_time` | double | sampling time for points in predicted path |

## Assumptions / Known limits

TBD.

## Reference

1. M. Werling, J. Ziegler, S. Kammel, and S. Thrun, “Optimal trajectory generation for dynamic street scenario in a frenet frame,” IEEE International Conference on Robotics and Automation, Anchorage, Alaska, USA, May 2010.
2. A. Houenou, P. Bonnifait, V. Cherfaoui, and Wen Yao, “Vehicle trajectory prediction based on motion model and maneuver recognition,” in 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, nov 2013, pp. 4363–4369.
