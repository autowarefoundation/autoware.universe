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

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <unordered_map>

namespace euclidean_cluster
{
VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster()
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size)
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size, float tolerance,
  float voxel_leaf_size, int min_points_number_per_voxel)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size),
  tolerance_(tolerance),
  voxel_leaf_size_(voxel_leaf_size),
  min_points_number_per_voxel_(min_points_number_per_voxel)
{
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  (void)pointcloud;
  (void)clusters;
  return false;
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg,
  std::vector<sensor_msgs::msg::PointCloud2> & clusters)
{
  // TODO(Saito) implement use_height is false version

  // create voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  int point_step = pointcloud_msg->point_step;
  int x_offset = pointcloud_msg->fields[pcl::getFieldIndex(*pointcloud_msg, "x")].offset;
  int y_offset = pointcloud_msg->fields[pcl::getFieldIndex(*pointcloud_msg, "y")].offset;
  int z_offset = pointcloud_msg->fields[pcl::getFieldIndex(*pointcloud_msg, "z")].offset;
  for (size_t i = 0; i < pointcloud_msg->data.size(); i += point_step) {
    pcl::PointXYZ point;
    std::memcpy(&point.x, &pointcloud_msg->data[i + x_offset], sizeof(float));
    std::memcpy(&point.y, &pointcloud_msg->data[i + y_offset], sizeof(float));
    std::memcpy(&point.z, &pointcloud_msg->data[i + z_offset], sizeof(float));
    pointcloud->push_back(point);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 100000.0);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // voxel is pressed 2d
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;
    pointcloud_2d_ptr->push_back(point2d);
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(1);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // create map to search cluster index from voxel grid index
  std::unordered_map</* voxel grid index */ int, /* cluster index */ int> map;
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      map[point_idx] = cluster_idx;
    }
  }

  // create vector of point cloud cluster. vector index is voxel grid index.
  std::vector<sensor_msgs::msg::PointCloud2> temporary_clusters;  // no check about cluster size
  std::vector<size_t> clusters_data_size;
  temporary_clusters.resize(cluster_indices.size());
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    temporary_clusters.at(i).height = pointcloud_msg->height;
    temporary_clusters.at(i).point_step = point_step;
    temporary_clusters.at(i).data.resize(pointcloud_msg->data.size());
    clusters_data_size.push_back(0);
  }
  for (size_t i = 0; i < pointcloud_msg->data.size(); i += point_step) {
    pcl::PointXYZ point;
    std::memcpy(&point.x, &pointcloud_msg->data[i + x_offset], sizeof(float));
    std::memcpy(&point.y, &pointcloud_msg->data[i + y_offset], sizeof(float));
    std::memcpy(&point.z, &pointcloud_msg->data[i + z_offset], sizeof(float));

    const int index =
      voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
    if (map.find(index) != map.end()) {
      std::memcpy(
        &temporary_clusters.at(map[index]).data[clusters_data_size.at(map[index])],
        &pointcloud_msg->data[i], point_step);
      clusters_data_size.at(map[index]) += point_step;
    }
  }

  // build output and check cluster size
  {
    for (size_t i = 0; i < temporary_clusters.size(); ++i) {
      if (!(min_cluster_size_ <= static_cast<int>(clusters_data_size.at(i) / point_step) &&
            static_cast<int>(clusters_data_size.at(i) / point_step) <= max_cluster_size_)) {
        continue;
      }
      temporary_clusters.at(i).data.resize(clusters_data_size.at(i));
      clusters.push_back(temporary_clusters[i]);
      clusters.back().header = pointcloud_msg->header;
      clusters.back().height = pointcloud_msg->height;
      clusters.back().width =
        temporary_clusters[i].data.size() / point_step / pointcloud_msg->height;
      clusters.back().is_dense = pointcloud_msg->is_dense;
      clusters.back().is_bigendian = pointcloud_msg->is_bigendian;
      clusters.back().fields = pointcloud_msg->fields;
      clusters.back().row_step = temporary_clusters[i].data.size() / pointcloud_msg->height;
    }
  }

  return true;
}

}  // namespace euclidean_cluster
