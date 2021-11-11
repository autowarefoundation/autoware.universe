# tier4_pcl_extensions

## Purpose

The `tier4_pcl_extensions` is a pcl extension library. The voxel grid filter in this package works with a different algorithm than the original one.

## Inner-workings / Algorithms

### Original Algorithm [1]

1. create a 3D voxel grid over the input pointcloud data
2. calculate centroid in each voxel
3. all the points are approximated with their centroid

### Extended Algorithm

1. create a 3D voxel grid over the input pointcloud data
2. calculate centroid in each voxel
3. **all the points are approximated with the closest point to their centroid**

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

<!-- Write inputs/outputs of this package.

Example:
  ### Input

  | Name                 | Type                                                | Description          |
  | -------------------- | --------------------------------------------------- | -------------------- |
  | `~/input/trajectory` | `autoware_planning_msgs::msg::Trajectory`           | reference trajectory |
  | `~/input/obstacles`  | `autoware_perception_msgs::msg::DynamicObjectArray` | obstacles            |

  ### Output

  | Name                  | Type                                      | Description         |
  | --------------------- | ----------------------------------------- | ------------------- |
  | `~/output/trajectory` | `autoware_planning_msgs::msg::Trajectory` | modified trajectory |
-->

## Parameters

<!-- Write parameters of this package.

Example:
  ### Node Parameters

  | Name                   | Type | Description                     |
  | ---------------------- | ---- | ------------------------------- |
  | `output_debug_markers` | bool | whether to output debug markers |

  ### Core Parameters

  | Name                 | Type   | Description                                                          |
  | -------------------- | ------ | -------------------------------------------------------------------- |
  | `min_object_size_m`  | double | minimum object size to be selected as avoidance target obstacles [m] |
  | `avoidance_margin_m` | double | avoidance margin to obstacles [m]                                    |
-->

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

[1] <https://pointclouds.org/documentation/tutorials/voxel_grid.html>

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
