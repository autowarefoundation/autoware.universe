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
#pragma once
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include "autoware_perception_msgs/TrafficLightRoiArray.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "sensor_msgs/Image.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <memory>
#include <mutex>

namespace traffic_light
{
class TrafficLightRoiVisualizerNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void connectCb();

  void imageRoiCallback(
    const sensor_msgs::ImageConstPtr & input_image_msg,
    const autoware_perception_msgs::TrafficLightRoiArray::ConstPtr & input_tl_roi_msg);

  void imageRoughRoiCallback(
    const sensor_msgs::ImageConstPtr & input_image_msg,
    const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_tl_roi_msg,
    const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_tl_rough_roi_msg);

private:
  bool createRect(
    cv::Mat& image,
    const autoware_perception_msgs::TrafficLightRoi& tl_roi,
    const cv::Scalar& color);

  ros::NodeHandle nh_, pnh_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_perception_msgs::TrafficLightRoiArray> roi_sub_;
  message_filters::Subscriber<autoware_perception_msgs::TrafficLightRoiArray> rough_roi_sub_;
  image_transport::Publisher image_pub_;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, autoware_perception_msgs::TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    autoware_perception_msgs::TrafficLightRoiArray,
    autoware_perception_msgs::TrafficLightRoiArray>
    SyncPolicyWithRoughRoi;
  typedef message_filters::Synchronizer<SyncPolicyWithRoughRoi> SyncWithRoughRoi;
  std::shared_ptr<SyncWithRoughRoi> sync_with_rough_roi_;

  std::mutex connect_mutex_;
  bool enable_fine_detection_;
};

}  // namespace traffic_light
