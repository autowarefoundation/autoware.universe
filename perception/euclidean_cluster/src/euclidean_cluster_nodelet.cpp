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
#include "autoware_perception_msgs/DynamicObjectWithFeature.h"
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"
#include "euclidean_cluster/euclidean_cluster_nodelet.hpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pluginlib/class_list_macros.h"
#include "sensor_msgs/point_cloud2_iterator.h"

namespace euclidean_cluster
{
EuclideanClusterNodelet::EuclideanClusterNodelet() {}

void EuclideanClusterNodelet::onInit()
{
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
  private_nh_.param<bool>("use_height", use_height_, false);
  private_nh_.param<int>("min_cluster_size", min_cluster_size_, 3);
  private_nh_.param<int>("max_cluster_size", max_cluster_size_, 200);
  private_nh_.param<float>("tolerance", tolerance_, 1.0);

  nh_ = getNodeHandle();
  pointcloud_sub_ =
    private_nh_.subscribe("input", 1, &EuclideanClusterNodelet::pointcloudCallback, this);

  cluster_pub_ =
    private_nh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("output", 10);
  debug_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("debug/clusters", 1);
}

void EuclideanClusterNodelet::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr & input_msg)
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
    autoware_perception_msgs::DynamicObjectWithFeatureArray output;
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
      sensor_msgs::PointCloud2 ros_pointcloud;
      autoware_perception_msgs::DynamicObjectWithFeature feature_object;
      pcl::toROSMsg(*cloud_cluster, ros_pointcloud);
      ros_pointcloud.header = input_msg->header;
      feature_object.feature.cluster = ros_pointcloud;
      output.feature_objects.push_back(feature_object);
    }
    cluster_pub_.publish(output);
  }

  // build debug msg
  if (debug_pub_.getNumSubscribers() < 1) return;
  {
    sensor_msgs::PointCloud2 pointcloud_output;

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
    debug_pub_.publish(pointcloud_output);
  }
}

}  // namespace euclidean_cluster

PLUGINLIB_EXPORT_CLASS(euclidean_cluster::EuclideanClusterNodelet, nodelet::Nodelet)
