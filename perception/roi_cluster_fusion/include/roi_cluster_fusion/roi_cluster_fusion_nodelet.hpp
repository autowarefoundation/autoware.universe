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
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "autoware_perception_msgs/DynamicObjectWithFeature.h"
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"
#include "message_filters/pass_through.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "sensor_msgs/CameraInfo.h"

namespace roi_cluster_fusion
{
class RoiClusterFusionNodelet : public nodelet::Nodelet
{
public:
  RoiClusterFusionNodelet();

private:
  virtual void onInit();

  void fusionCallback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_cluster_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi0_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi1_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi2_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi3_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi4_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi5_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi6_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi7_msg);
  void cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr & input_camera_info_msg, const int id);
  double calcIoU(
    const sensor_msgs::RegionOfInterest & roi_1, const sensor_msgs::RegionOfInterest & roi_2);
  double calcIoUX(
    const sensor_msgs::RegionOfInterest & roi_1, const sensor_msgs::RegionOfInterest & roi_2);
  double calcIoUY(
    const sensor_msgs::RegionOfInterest & roi_1, const sensor_msgs::RegionOfInterest & roi_2);

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher labeled_cluster_pub_;
  std::vector<std::shared_ptr<ros::Subscriber> > v_camera_info_sub_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray> cluster_sub_;
  std::vector<std::shared_ptr<
    message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray> > >
    v_roi_sub_;
  message_filters::PassThrough<autoware_perception_msgs::DynamicObjectWithFeatureArray>
    passthrough_;
  typedef message_filters::sync_policies::ApproximateTime<
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray,
    autoware_perception_msgs::DynamicObjectWithFeatureArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_ptr_;
  inline void dummyCallback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input)
  {
    autoware_perception_msgs::DynamicObjectWithFeatureArray dummy;
    dummy.header.stamp = input->header.stamp;
    passthrough_.add(
      boost::make_shared<autoware_perception_msgs::DynamicObjectWithFeatureArray>(dummy));
  }
  // ROS Parameters
  bool use_iou_x_;
  bool use_iou_y_;
  bool use_iou_;
  bool use_cluster_semantic_type_;
  double iou_threshold_;
  int rois_number_;
  std::map<int, sensor_msgs::CameraInfo> m_camera_info_;
};

}  // namespace roi_cluster_fusion
