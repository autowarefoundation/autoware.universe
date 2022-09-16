// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <unordered_map>

namespace euclidean_cluster
{
VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster() {}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size)
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size, float tolerance,
  float voxel_leaf_size, int min_points_number_per_voxel)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size, tolerance),
  voxel_leaf_size_(voxel_leaf_size),
  min_points_number_per_voxel_(min_points_number_per_voxel)
{
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  setPointcloud(pointcloud, pointcloud_ptr);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> temporary_clusters;  // no check about cluster size
  solveVoxelBasedClustering(pointcloud_ptr, temporary_clusters);

  // build output and check cluster size
  {
    for (const auto & cluster : temporary_clusters) {
      if (!(params_.min_cluster_size <= static_cast<int>(cluster.points.size()) &&
            static_cast<int>(cluster.points.size()) <= params_.max_cluster_size)) {
        continue;
      }
      clusters.emplace_back(cluster);
      clusters.back().width = cluster.points.size();
      clusters.back().height = 1;
      clusters.back().is_dense = false;
    }
  }

  return true;
}

void VoxelGridBasedEuclideanCluster::setPointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud_ptr)
{
  // create voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 100000.0);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // voxel is pressed 2d if use_height is false.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point3d;
    point3d.x = point.x;
    point3d.y = point.y;
    point3d.z = params_.use_height ? point.z : 0.0;
    pc_ptr->emplace_back(point3d);
  }
  pointcloud_ptr = pc_ptr;
}

void VoxelGridBasedEuclideanCluster::solveVoxelBasedClustering(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud_ptr,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & temporary_clusters)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  std::vector<pcl::PointIndices> cluster_indices;
  solveEuclideanClustering(pcl_euclidean_cluster, cluster_indices, pointcloud_ptr);

  // create map to search cluster index from voxel grid index
  std::unordered_map</* voxel grid index */ int, /* cluster index */ int> map;
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      map[point_idx] = cluster_idx;
    }
  }

  // create vector of point cloud cluster. vector index is voxel grid index.
  temporary_clusters.resize(cluster_indices.size());
  for (const auto & point : pointcloud_ptr->points) {
    const int index =
      voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
    if (map.find(index) != map.end()) {
      temporary_clusters.at(map[index]).points.emplace_back(point);
    }
  }
}

}  // namespace euclidean_cluster
