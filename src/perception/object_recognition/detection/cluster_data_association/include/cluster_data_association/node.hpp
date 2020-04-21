/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cluster_data_association/data_association.hpp>
#include <memory>
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

namespace cluster_data_association
{
class ClusterDataAssociationNode
{
public:
  ClusterDataAssociationNode();
  ~ClusterDataAssociationNode(){};

private:
  void clusterCallback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_cluster0_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_cluster1_msg);

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher associated_cluster_pub_;
  message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray>
    cluster0_sub_;
  message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray>
    cluster1_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  DataAssociation data_association_;

  // ROS Parameters
};

}  // namespace cluster_data_association
