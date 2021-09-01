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
#include "autoware_perception_msgs/msg/dynamic_object_with_feature.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "euclidean_cluster/voxel_grid_based_euclidean_cluster_nodelet.hpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <unordered_map>

namespace euclidean_cluster
{
VoxelGridBasedEuclideanClusterNodelet::VoxelGridBasedEuclideanClusterNodelet(
  const rclcpp::NodeOptions & options)
: Node("voxel_grid_based_euclidean_cluster_node", options)
{

  target_frame_ = this->declare_parameter("target_frame", "base_link");
//  bool use_height_ = this->declare_parameter("use_height", false);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 1);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 500);
  tolerance_ = this->declare_parameter("tolerance", 1.0);
  voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.5);
  min_points_number_per_voxel_ = this->declare_parameter("min_points_number_per_voxel", 3);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&VoxelGridBasedEuclideanClusterNodelet::pointcloudCallback, this, _1));

  cluster_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "output",
    10);
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);
}

void VoxelGridBasedEuclideanClusterNodelet::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);

  // create voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 100000.0);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel_);
  voxel_grid_.setInputCloud(raw_pointcloud_ptr);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // voxel is pressed 2d
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < voxel_map_ptr->points.size(); ++i) {
    pcl::PointXYZ point;
    point.x = voxel_map_ptr->points.at(i).x;
    point.y = voxel_map_ptr->points.at(i).y;
    point.z = 0.0;
    pointcloud_2d_ptr->push_back(point);
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
  int cluster_idx = 0;
  for (std::vector<pcl::PointIndices>::const_iterator cluster_itr = cluster_indices.begin();
    cluster_itr != cluster_indices.end(); ++cluster_itr)
  {
    for (std::vector<int>::const_iterator point_itr = cluster_itr->indices.begin();
      point_itr != cluster_itr->indices.end(); ++point_itr)
    {
      map[*point_itr] = cluster_idx;
    }
    ++cluster_idx;
  }

  // create vector of point cloud cluster. vector index is voxel grid index.
  std::vector<pcl::PointCloud<pcl::PointXYZ>> v_cluster;
  v_cluster.resize(cluster_idx);
  for (size_t i = 0; i < raw_pointcloud_ptr->points.size(); ++i) {
    const int index = voxel_grid_.getCentroidIndexAt(
      voxel_grid_.getGridCoordinates(
        raw_pointcloud_ptr->points.at(i).x, raw_pointcloud_ptr->points.at(i).y,
        raw_pointcloud_ptr->points.at(i).z));
    if (map.find(index) != map.end()) {
      v_cluster.at(map[index]).points.push_back(raw_pointcloud_ptr->points.at(i));
    }
  }

  // build output msg
  {
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output;
    output.header = input_msg->header;
    for (std::vector<pcl::PointCloud<pcl::PointXYZ>>::const_iterator cluster_itr =
      v_cluster.begin();
      cluster_itr != v_cluster.end(); ++cluster_itr)
    {
      if (!(min_cluster_size_ <= static_cast<int>(cluster_itr->points.size()) &&
        static_cast<int>(cluster_itr->points.size()) <= max_cluster_size_))
      {
        continue;
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (pcl::PointCloud<pcl::PointXYZ>::const_iterator point_itr = cluster_itr->points.begin();
        point_itr != cluster_itr->points.end(); ++point_itr)
      {
        cloud_cluster->points.push_back(*point_itr);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = false;
      sensor_msgs::msg::PointCloud2 ros_pointcloud;
      autoware_perception_msgs::msg::DynamicObjectWithFeature feature_object;
      pcl::toROSMsg(*cloud_cluster, ros_pointcloud);
      ros_pointcloud.header = input_msg->header;
      feature_object.feature.cluster = ros_pointcloud;
      feature_object.object.state.pose_covariance.pose.position = getCentroid(ros_pointcloud);
      output.feature_objects.push_back(feature_object);
    }
    cluster_pub_->publish(output);
  }

  // build debug msg
  if (debug_pub_->get_subscription_count() < 1) {return;}
  {
    sensor_msgs::msg::PointCloud2 pointcloud_output;

    int i = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<pcl::PointCloud<pcl::PointXYZ>>::const_iterator cluster_itr =
      v_cluster.begin();
      cluster_itr != v_cluster.end(); ++cluster_itr)
    {
      if (!(min_cluster_size_ <= static_cast<int>(cluster_itr->points.size()) &&
        static_cast<int>(cluster_itr->points.size()) <= max_cluster_size_))
      {
        continue;
      }
      for (pcl::PointCloud<pcl::PointXYZ>::const_iterator point_itr = cluster_itr->points.begin();
        point_itr != cluster_itr->points.end(); ++point_itr)
      {
        pcl::PointXYZI point;
        point.x = point_itr->x;
        point.y = point_itr->y;
        point.z = point_itr->z;
        point.intensity = (float)(((i % 10) + 1) / 10.f);
        cloud_cluster->points.push_back(point);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = false;
      ++i;
    }
    pcl::toROSMsg(*cloud_cluster, pointcloud_output);
    pointcloud_output.header = input_msg->header;
    debug_pub_->publish(pointcloud_output);
  }
}

geometry_msgs::msg::Point VoxelGridBasedEuclideanClusterNodelet::getCentroid(
  const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.f;
  centroid.y = 0.f;
  centroid.z = 0.f;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"),
    iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    centroid.x += *iter_x;
    centroid.y += *iter_y;
    centroid.z += *iter_z;
  }
  centroid.x = centroid.x / ((float)pointcloud.height * (float)pointcloud.width);
  centroid.y = centroid.y / ((float)pointcloud.height * (float)pointcloud.width);
  centroid.z = centroid.z / ((float)pointcloud.height * (float)pointcloud.width);
  return centroid;
}

}  // namespace euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(euclidean_cluster::VoxelGridBasedEuclideanClusterNodelet)
