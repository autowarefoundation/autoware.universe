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

#include "euclidean_cluster/euclidean_cluster_interface.hpp"

namespace euclidean_cluster
{
void EuclideanClusterInterface::solveEuclideanClustering(
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> & pcl_euclidean_cluster,
  std::vector<pcl::PointIndices> & cluster_indices,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud_ptr)
{
  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);

  // clustering
  pcl_euclidean_cluster.setClusterTolerance(params_.tolerance);
  pcl_euclidean_cluster.setMinClusterSize(params_.min_cluster_size);
  pcl_euclidean_cluster.setMaxClusterSize(params_.max_cluster_size);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);
}
}  // namespace euclidean_cluster
