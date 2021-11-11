# livox_tag_filter

## Purpose

The `livox_tag_filter` is a node that removes noise from pointcloud by using the following tags:

- Point property based on spatial position
- Point property based on intensity
- Return number

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

| Name      | Type                            | Description      |
| --------- | ------------------------------- | ---------------- |
| `~/input` | `sensor_msgs::msg::PointCloud2` | reference points |

### Output

| Name       | Type                            | Description     |
| ---------- | ------------------------------- | --------------- |
| `~/output` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Node Parameters

| Name          | Type            | Description                            |
| ------------- | --------------- | -------------------------------------- |
| `ignore_tags` | vector<int64_t> | ignored tags (See the following table) |

### Tag Parameters

| Bit | Description                              | Options                                    |
| --- | ---------------------------------------- | ------------------------------------------ |
| 0~1 | Point property based on spatial position | 00: Normal                                 |
|     |                                          | 01: High confidence level of the noise     |
|     |                                          | 10: Moderate confidence level of the noise |
|     |                                          | 11: Low confidence level of the noise      |
| 2~3 | Point property based on intensity        | 00: Normal                                 |
|     |                                          | 01: High confidence level of the noise     |
|     |                                          | 10: Moderate confidence level of the noise |
|     |                                          | 11: Reserved                               |
| 4~5 | Return number                            | 00: return 0                               |
|     |                                          | 01: return 1                               |
|     |                                          | 10: return 2                               |
|     |                                          | 11: return 3                               |
| 6~7 | Reserved                                 |                                            |

You can download more detail description about the livox from external link [1].

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

[1] <https://www.livoxtech.com/downloads>

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
