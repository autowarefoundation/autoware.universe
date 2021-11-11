# gnss_poser

## Purpose

The `gnss_poser` is a node that subscribes gnss sensing messages and calculates vehicle pose with covariance.

## Inner-workings / Algorithms

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

| Name             | Type                          | Description                                                                                     |
| ---------------- | ----------------------------- | ----------------------------------------------------------------------------------------------- |
| `~/input/fix`    | `sensor_msgs::msg::NavSatFix` | gnss status message                                                                             |
| `~/input/navpvt` | `ublox_msgs::msg::NavPVT`     | position, velocity and time solution (You can see detail description in reference document [1]) |

### Output

| Name                     | Type                                            | Description                                                    |
| ------------------------ | ----------------------------------------------- | -------------------------------------------------------------- |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`               | vehicle pose calculated from gnss sensing data                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | vehicle pose with covariance calculated from gnss sensing data |
| `~/output/gnss_fixed`    | `autoware_debug_msgs::msg::BoolStamped`         | gnss fix status                                                |

## Parameters

### Core Parameters

| Name                 | Type   | Default Value    | Description                                                                                     |
| -------------------- | ------ | ---------------- | ----------------------------------------------------------------------------------------------- |
| `base_frame`         | string | "base_link"      | frame d                                                                                         |
| `gnss_frame`         | string | "gnss"           | frame id                                                                                        |
| `gnss_base_frame`    | string | "gnss_base_link" | frame id                                                                                        |
| `map_frame`          | string | "map"            | frame id                                                                                        |
| `use_ublox_receiver` | bool   | false            | flag to use ublox receiver                                                                      |
| `plane_zone`         | int    | 9                | identification number of the plane rectangular coordinate systems (See, reference document [2]) |

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

[1] <https://github.com/KumarRobotics/ublox.git>

[2] <https://www.gsi.go.jp/LAW/heimencho.html>

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
