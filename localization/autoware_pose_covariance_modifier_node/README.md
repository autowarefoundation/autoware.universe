# Overview

This package is used to enable GNSS and NDT to be used together in localization.
GNSS and NDT are different sources that generate poses for use in EKF. For EKF,
the covariance values of the pose source are important. The GNSS system can provide
us with its own error (RMSE) values. Using these error values, covariance values for
GNSS can be calculated. However, in default autoware, NDT covariance values are determined
by default values. While this package manages which pose source will be used by reference
to the error values coming from GNSS, it also manages situations where GNSS and NDT are used together.

## Flowchart

- Default Autoware Pose Source is only NDT:

<p align="center">
<img src="./media/new_proposal-original.drawio.png" width="320">
</p>

- You can see how autoware_pose_covariance_modifier_node works in the diagram below:

<p align="center">
<img src="./media/new_proposal-proposal-extended-proposal.drawio.png" width="620">
</p>

## Activate this feature

This package is not used by default autoware, you need to activate it to use it.

To activate, you need to change the `use_autoware_pose_covariance_modifier` parameter to true within
the [pose_twist_estimator.launch.xml](https://github.com/meliketanrikulu/autoware.universe/blob/0c14cc97d563c77262f74e306916a9cd26992e73/launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml#L3).

## Node

### Subscribed topics

| Name                             | Type                                            | Description            |
| -------------------------------- | ----------------------------------------------- | ---------------------- |
| `input_gnss_pose_with_cov_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` | Input GNSS pose topic. |
| `input_ndt_pose_with_cov_topic`  | `geometry_msgs::msg::PoseWithCovarianceStamped` | Input NDT pose topic.  |

### Published topics

| Name                                | Type                                            | Description                                                                                             |
| ----------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| `output_pose_with_covariance_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` | Output pose topic. It will be sent as input to ekf_localizer package.                                   |
| `selected_pose_type`                | `std_msgs::msg::String`                         | Declares which pose sources are used in the output of this package                                      |
| `output/ndt_position_rmse`          | `std_msgs::msg::Float32`                        | Output pose ndt average rmse in position xy. It is published only when the enable_debug_topics is true. |
| `output/gnss_position_rmse`         | `std_msgs::msg::Float32`                        | Output pose gnss average rmse in position xy.It is published only when the enable_debug_topics is true. |

### Parameters

| Name                                         | Type     | Description                                                                    |
| -------------------------------------------- | -------- | ------------------------------------------------------------------------------ |
| `error_thresholds.gnss_error_reliable_max`   | `double` | Threshold value for the range in which GNSS error is most reliable.            |
| `error_thresholds.gnss_error_unreliable_min` | `double` | Threshold value at which GNSS error is not considered reliable.                |
| `error_thresholds.yaw_error_deg_threshold`   | `double` | Threshold value to understand whether the yaw error is within reliable limits. |
| `gnss_pose_timeout_sec`                      | `double` | Maximum waiting time when message is delayed from gnss pose source             |
| `debug.enable_debug_topics`                  | `bool`   | Enables the debug topics                                                       |

## Important notes

- In order to use this package, your GNSS sensor must provide you with the error value. If you do not have a GNSS sensor
  that provides you with the error value, you cannot use this package.
- You need to use this package with georeferenced map
