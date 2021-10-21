# pointcloud_preprocessor : occupancy_grid_map_outlier_filter

## Purpose

This node is an outlier filter based on a occupancy grid map.
Depending on the implementation of occupancy grid map, it can be called an outlier filter in time series, since the occupancy grid map expresses the occupancy probabilities in time series.

## Inner-workings / Algorithms

1. Use the occupancy grid map to separate point clouds into those with low occupancy probability and those with high occupancy probability.

2. If `use_radius_search_2d_filter` is true, then apply an radius search 2d outlier filter to the point cloud that is determined to have a low occupancy probability.
   1. For each point, determine the outlier from the radius (`radius_search_2d_filter/search_radius`) and the number of point clouds.
   2. The number of point clouds can be multiplied by `radius_search_2d_filter/min_points_and_distance_ratio` and distance from base link. However, the minimum and maximum number of point clouds is limited.

## Inputs / Outputs

### Input

| Name                         | Type                      | Description                                                                                |
| ---------------------------- | ------------------------- | ------------------------------------------------------------------------------------------ |
| `~/input/pointcloud`         | `sensor_msgs/PointCloud2` | Obstacle point cloud with ground removed.                                                  |
| `~/input/occupancy_grid_map` | `nav_msgs/OccupancyGrid`  | A map in which the probability of the presence of an obstacle is occupancy probability map |

### Output

| Name                                        | Type                      | Description                                                                                                                  |
| ------------------------------------------- | ------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `~/output/pointcloud`                       | `sensor_msgs/PointCloud2` | Point cloud with outliers removed. trajectory                                                                                |
| `~/output/debug/outlier/pointcloud`         | `sensor_msgs/PointCloud2` | Point clouds removed as outliers.                                                                                            |
| `~/output/debug/low_confidence/pointcloud`  | `sensor_msgs/PointCloud2` | Point clouds that had a low probability of occupancy in the occupancy grid map. However, it is not considered as an outlier. |
| `~/output/debug/high_confidence/pointcloud` | `sensor_msgs/PointCloud2` | Point clouds that had a high probability of occupancy in the occupancy grid map. trajectory                                  |

## Parameters

| Name                                                    | Type   | Description                                                                                                                                                                                                                    |
| ------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `map_frame`                                             | string | map frame id                                                                                                                                                                                                                   |
| `base_link_frame`                                       | string | base link frame id                                                                                                                                                                                                             |
| `cost_threshold`                                        | int    | Cost threshold of occupancy grid map (0~100). 100 means 100% probability that there is an obstacle, close to 50 means that it is indistinguishable whether it is an obstacle or free space, 0 means that there is no obstacle. |
| `enable_debugger`                                       | bool   | Whether to output the point cloud for debugging.                                                                                                                                                                               |
| `use_radius_search_2d_filter`                           | bool   | Whether or not to apply density-based outlier filters to objects that are judged to have low probability of occupancy on the occupancy grid map.                                                                               |
| `radius_search_2d_filter/search_radius`                 | float  | Radius when calculating the density                                                                                                                                                                                            |
| `radius_search_2d_filter/min_points_and_distance_ratio` | float  | Threshold value of the number of point clouds per radius when the distance from baselink is 1m, because the number of point clouds varies with the distance from baselink.                                                     |
| `radius_search_2d_filter/min_points`                    | int    | Minimum number of point clouds per radius                                                                                                                                                                                      |
| `radius_search_2d_filter/max_points`                    | int    | Maximum number of point clouds per radius                                                                                                                                                                                      |

<!-- ## Assumptions / Known limits -->

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

<!-- ## (Optional) Error detection and handling -->

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

<!-- ## (Optional) Performance characterization -->

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

- [RadiusSearchOutlier](https://vml.sakura.ne.jp/koeda/PCL/tutorials/html/remove_outliers.html)

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
