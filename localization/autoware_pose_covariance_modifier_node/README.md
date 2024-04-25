# Overview

This package is used to enable GNSS and NDT to be used together in localization.
GNSS and NDT are different sources that generate poses for use in EKF. For EKF,
the covariance values of the pose source are important. The GNSS system can provide
us with its own "standard deviation" values. Using these values, covariance values for
GNSS can be calculated. However, in default autoware, NDT covariance values are determined
by default values. While this package manages which pose source will be used by reference
to the standard deviation values coming from GNSS, it also manages situations where GNSS and NDT are used together.

## Flowchart

- Default Autoware Pose Source is only NDT:

<p align="center">
<img src="./media/new_proposal-original.drawio.png" width="320">
</p>

- You can see how autoware_pose_covariance_modifier_node works in the diagram below:

<p align="center">
<img src="./media/new_proposal-proposal-extended-proposal.drawio.png" width="620">
</p>

### _How does the "Interpolate GNSS and NDT pose" part work ?_

The goal is to reduce the NDT covariance as the GNSS covariance increases, aiming for a smoother and more balanced transition between NDT and and maximize the benefit from GNSS poses.

<p align="center">
<img src="./media/ndt_stddev_calculation_formula.drawio.png" width="720">
</p>

<p align="center">
<img src="./media/formula.drawio.png" width="520">
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

| Name                                | Type                                            | Description                                                                                                           |
| ----------------------------------- | ----------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `output_pose_with_covariance_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` | Output pose topic. It will be sent as input to ekf_localizer package.                                                 |
| `selected_pose_type`                | `std_msgs::msg::String`                         | Declares which pose sources are used in the output of this package                                                    |
| `output/ndt_position_stddev`        | `std_msgs::msg::Float32`                        | Output pose ndt average standard deviation in position xy. It is published only when the enable_debug_topics is true. |
| `output/gnss_position_stddev`       | `std_msgs::msg::Float32`                        | Output pose gnss average standard deviation in position xy.It is published only when the enable_debug_topics is true. |

### Parameters

The parameters are set in config/autoware_pose_covariance_modifier.param.yaml .

#### For Standard Deviation thresholds

{{ json_to_markdown("localization/autoware_pose_covariance_modifier_node/schema/sub/stddev_thresholds.sub_schema.json") }}

#### For validation

{{ json_to_markdown("localization/autoware_pose_covariance_modifier_node/schema/sub/validation.sub_schema.json") }}

#### For debug

{{ json_to_markdown("localization/autoware_pose_covariance_modifier_node/schema/sub/debug.sub_schema.json") }}

## Important notes

- In order to use this package, your GNSS sensor must provide you with the standard deviation or variance value. If you
  do not have a GNSS sensor
  that provides you with these values, you cannot use this package.
- You need to use this package with georeferenced map.
