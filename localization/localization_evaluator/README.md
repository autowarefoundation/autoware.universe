# localization_evaluator

## Purpose

`localization_evaluator` is the package to evaluate localization output. The Error is published as `PoseStamped` message and can be visualized using plotjuggler. 

## Inputs / Outputs

### Input

| Name                                          | Type                              | Description       |
| --------------------------------------------- | --------------------------------- | ----------------- |
| `/localization/pose_twist_fusion_filter/pose` | `geometry_msgs::msg::PoseStamped` | pose from vehicle |
| `/ground_truth`                               | `geometry_msgs::msg::PoseStamped` | ground truth pose |

### Output

| Name                   | Type                              | Description         |
| ---------------------- | --------------------------------- | ------------------- |
| `/relative_pose_error` | `geometry_msgs::msg::PoseStamped` | relative pose error |

## Parameters

| Parameter    | Type   | Description                                                     |
| ------------ | ------ | --------------------------------------------------------------- |
| `time_delta` | Double | timestamp difference for vehicle pose and ground truth matching |

## Assumptions / Known limits

This module uses RPE(Relative Pose Error) as an evaluation metric. To put simply, a subset of trajectory, a pair of states in this case , is used to compute a relative error. This corresponds to the drift of defined trajectory. One know issue is that we cannot perfectly align a ground truth pose with an output of localization due to unknown processing time and time synchronization of different nodes. Therefore, we interpolate poses from previous and post outputs from localization. 

Reference:
[A_benchmark_for_the_evaluation_of_RGB-D_SLAM_systems](https://www.researchgate.net/publication/261353760_A_benchmark_for_the_evaluation_of_RGB-D_SLAM_systems)
