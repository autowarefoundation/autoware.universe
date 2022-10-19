// Copyright 2021 Tier IV, Inc.
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

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>

namespace euclidean_cluster
{
struct ClusterParameters
{
  bool use_height;
  int min_cluster_size;
  int max_cluster_size;
  float tolerance;
};  // stuct ClusterParameters

class EuclideanClusterInterface
{
public:
  EuclideanClusterInterface() = default;
  EuclideanClusterInterface(bool use_height, int min_cluster_size, int max_cluster_size)
  : params_{use_height, min_cluster_size, max_cluster_size, 0.7}
  {
  }
  EuclideanClusterInterface(
    bool use_height, int min_cluster_size, int max_cluster_size, float tolerance)
  : params_{use_height, min_cluster_size, max_cluster_size, tolerance}
  {
  }
  virtual ~EuclideanClusterInterface() = default;
  void setUseHeight(bool use_height) { params_.use_height = use_height; }
  void setMinClusterSize(int size) { params_.min_cluster_size = size; }
  void setMaxClusterSize(int size) { params_.max_cluster_size = size; }
  void setTolerance(float tolerance) { params_.tolerance = tolerance; }
  virtual bool cluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters) = 0;
  void solveEuclideanClustering(
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> & pcl_euclidean_cluster,
    std::vector<pcl::PointIndices> & cluster_indices,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud_ptr);

protected:
  ClusterParameters params_;
  virtual void preparePointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud_ptr) = 0;
};  // class EuclideanClusterInterface

}  // namespace euclidean_cluster
