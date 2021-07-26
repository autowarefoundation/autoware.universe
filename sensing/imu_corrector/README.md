# imu_corrector

## Purpose

`imu_corrector_node` is a node that correct imu data.

1. Correct yaw rate offset by reading the parameter.
2. Correct yaw rate standard deviation by reading the parameter.

Use the value estimated by [deviation_estimator](https://github.com/tier4/calibration_tools/tree/feature/add_deviation_estimator_ros2/localization/deviation_estimation_tools) as the parameters for this node.

<!--
## Inner-workings / Algorithms
-->
<!-- Write how this package works. Flowcharts and figures are great. Add sub-sections as you like.

Example:
  ### Flowcharts

  ...(PlantUML or something)

  ### State Transitions

  ...(PlantUML or something)

  ### How to filter target obstacles

  ...

  ### How to optimize trajectory

  ...
-->

## Inputs / Outputs

### Input

| Name     | Type                    | Description  |
| -------- | ----------------------- | ------------ |
| `~input` | `sensor_msgs::msg::Imu` | raw imu data |

### Output

| Name      | Type                    | Description        |
| --------- | ----------------------- | ------------------ |
| `~output` | `sensor_msgs::msg::Imu` | corrected imu data |

## Parameters

### Core Parameters

| Name                         | Type   | Description                       |
| ---------------------------- | ------ | --------------------------------- |
| `angular_velocity_offset_z`  | double | yaw rate offset [rad]             |
| `angular_velocity_stddev_zz` | double | yaw rate standard deviation [rad] |

<!--
## Assumptions / Known limits
-->
<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

<!--
## (Optional) Error detection and handling
-->
<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

<!--
## (Optional) Performance characterization
-->
<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

<!--
## (Optional) References/External links
-->
<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->
<!--
## (Optional) Future extensions / Unimplemented parts
-->
<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
