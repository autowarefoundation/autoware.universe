# autoware_euclidean_cluster

## Purpose

autoware_euclidean_cluster is a package for clustering points into smaller parts to classify objects.

This package has two clustering methods: `euclidean_cluster` and `voxel_grid_based_euclidean_cluster`.

## Inner-workings / Algorithms

### euclidean_cluster

`pcl::EuclideanClusterExtraction` is applied to points. See [official document](https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html) for details.

### voxel_grid_based_euclidean_cluster

1. A centroid in each voxel is calculated by `pcl::VoxelGrid`.
2. The centroids are clustered by `pcl::EuclideanClusterExtraction`.
3. The input points are clustered based on the clustered centroids.

## Inputs / Outputs

### Input

| Name    | Type                            | Description      |
| ------- | ------------------------------- | ---------------- |
| `input` | `sensor_msgs::msg::PointCloud2` | input pointcloud |

### Output

| Name             | Type                                                     | Description                                  |
| ---------------- | -------------------------------------------------------- | -------------------------------------------- |
| `output`         | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | cluster pointcloud                           |
| `debug/clusters` | `sensor_msgs::msg::PointCloud2`                          | colored cluster pointcloud for visualization |

## Parameters

### Core Parameters

#### euclidean_cluster

{{ json_to_markdown("perception/autoware_euclidean_cluster/schema/euclidean_cluster.schema.json") }}

#### voxel_grid_based_euclidean_cluster

{{ json_to_markdown("perception/autoware_euclidean_cluster/schema/voxel_grid_based_euclidean_cluster.schema.json") }}

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

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts

The `use_height` option of `voxel_grid_based_euclidean_cluster` isn't implemented yet.
