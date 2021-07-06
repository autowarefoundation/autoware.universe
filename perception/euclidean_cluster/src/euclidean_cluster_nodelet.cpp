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
#include "euclidean_cluster/euclidean_cluster_nodelet.hpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace euclidean_cluster
{
EuclideanClusterNodelet::EuclideanClusterNodelet(const rclcpp::NodeOptions & options)
: Node("euclidean_cluster_node", options)
{
  target_frame_ = this->declare_parameter("target_frame", "base_link");
  use_height_ = this->declare_parameter("use_height", false);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 3);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 200);
  tolerance_ = this->declare_parameter("tolerance", 1.0);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1), std::bind(&EuclideanClusterNodelet::pointcloudCallback, this, _1));

  cluster_pub_ = this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>("output", 10);
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);
}

void EuclideanClusterNodelet::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (!use_height_) {
    for (size_t i = 0; i < raw_pointcloud_ptr->size(); ++i) {
      pcl::PointXYZ point;
      point.x = raw_pointcloud_ptr->points.at(i).x;
      point.y = raw_pointcloud_ptr->points.at(i).y;
      point.z = 0.0;
      pointcloud_ptr->push_back(point);
    }
  } else {
    pointcloud_ptr = raw_pointcloud_ptr;
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(min_cluster_size_);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // build output msg
  {
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output;
    output.header = input_msg->header;
    for (std::vector<pcl::PointIndices>::const_iterator cluster_itr = cluster_indices.begin();
         cluster_itr != cluster_indices.end(); ++cluster_itr) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator point_itr = cluster_itr->indices.begin();
           point_itr != cluster_itr->indices.end(); ++point_itr) {
        cloud_cluster->points.push_back(raw_pointcloud_ptr->points[*point_itr]);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
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
  if (debug_pub_->get_subscription_count() < 1) return;
  {
    sensor_msgs::msg::PointCloud2 pointcloud_output;

    int i = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<pcl::PointIndices>::const_iterator cluster_itr = cluster_indices.begin();
         cluster_itr != cluster_indices.end(); ++cluster_itr) {
      for (std::vector<int>::const_iterator point_itr = cluster_itr->indices.begin();
           point_itr != cluster_itr->indices.end(); ++point_itr) {
        pcl::PointXYZI point;
        point.x = raw_pointcloud_ptr->points[*point_itr].x;
        point.y = raw_pointcloud_ptr->points[*point_itr].y;
        point.z = raw_pointcloud_ptr->points[*point_itr].z;
        point.intensity = (float)i * 1.0 / (float)cluster_indices.size();
        cloud_cluster->points.push_back(point);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      ++i;
    }
    pcl::toROSMsg(*cloud_cluster, pointcloud_output);
    pointcloud_output.header = input_msg->header;
    debug_pub_->publish(pointcloud_output);
  }
}

geometry_msgs::msg::Point EuclideanClusterNodelet::getCentroid(
  const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.f;
  centroid.y = 0.f;
  centroid.z = 0.f;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"),
       iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
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

RCLCPP_COMPONENTS_REGISTER_NODE(euclidean_cluster::EuclideanClusterNodelet)
