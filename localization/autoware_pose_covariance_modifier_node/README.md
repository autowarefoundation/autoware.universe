# Autoware Pose Covariance Modifier Node

## Purpose

This package is used to enable GNSS and NDT to be used together in localization.

## Function

This package takes in GNSS (Global Navigation Satellite System) and NDT (Normal Distribution Transform) poses with covariances.
It outputs a single pose with an associated covariance:
- Directly the GNSS pose and its covariance.
- Directly the NDT pose and its covariance.
- A weighted average of both GNSS and NDT poses and covariances.

## Description

GNSS and NDT nodes provide the pose with covariance data utilized in an Extended Kalman Filter (EKF).

Accurate covariance values are crucial for the effectiveness of the EKF in estimating the state.

The GNSS system generates reliable standard deviation values, which can be transformed into covariance measures.

But we currently don't have a reliable way to determine the covariance values for the NDT poses.
And the NDT matching system in Autoware outputs poses with preset covariance values.

For this reason, this package is designed to manage the selection of the pose source,
based on the standard deviation values provided by the GNSS system.

It also tunes the covariance values of the NDT poses, based on the GNSS standard deviation values.

## Flowchart

### Without this package

<p align="center">
<img src="./media/new_proposal-original.drawio.png" width="320">
</p>

### With this package

<p align="center">
<img src="./media/new_proposal-proposal-extended-proposal.drawio.png" width="620">
</p>

## _How does the "Interpolate GNSS and NDT pose" part work ?_

In this section, both NDT and GNSS poses are published from the same topic and given as input
to [ekf_localizer](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ekf_localizer). In
other words, at
this stage, both NDT and GNSS are used in localization. Here, while the covariance values of the GNSS poses remain
exactly the same, the NDT covariance values are determined by reference to the GNSS covariances, provided that they are
kept within a certain range.
The goal is to reduce the NDT covariance as the GNSS covariance increases, aiming for a smoother and more balanced
transition between NDT and and maximize the benefit from GNSS poses. Here is how NDT covariance values are calculated

<p align="center">
<img src="./media/ndt_stddev_calculation_formula.drawio.png" width="720">
</p>

<p align="center">
<img src="./media/formula.drawio.png" width="520">
</p>

## How to use this package

> **This package is disabled by default in Autoware, you need to manually enable it.**

To enable this package, you need to change the `use_autoware_pose_covariance_modifier` parameter to `true` within
the [pose_twist_estimator.launch.xml](../../launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml#L3).

When you activate this feature, `autoware_pose_covariance_modifier_node` is run and only the message of
type `geometry_msgs::msg::PoseWithCovarianceStamped` in the output
of [ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher)
is renamed. In this way, while the ndt_scan_matcher output (the message of the
type `geometry_msgs::msg::PoseWithCovarianceStamped`)
enters the `autoware_pose_covariance_modifier_node` and the output of this package is given
to [ekf_localizer](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ekf_localizer).

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

{{ json_to_markdown("
localization/autoware_pose_covariance_modifier_node/schema/sub/stddev_thresholds.sub_schema.json") }}

#### For validation

{{ json_to_markdown("localization/autoware_pose_covariance_modifier_node/schema/sub/validation.sub_schema.json") }}

#### For debug

{{ json_to_markdown("localization/autoware_pose_covariance_modifier_node/schema/sub/debug.sub_schema.json") }}

> ## Important notes
>
> 1. In order to use this package, your GNSS sensor must provide you with the standard deviation or variance value. If you do not have a GNSS sensor that provides you with these values, you cannot use this package.
> 2. You need to use this package with georeferenced map.
