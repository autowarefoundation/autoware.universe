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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include "autoware_perception_msgs/TrafficLightRoiArray.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "sensor_msgs/Image.h"

#include <memory>
#include <string>

namespace traffic_light
{
class TrafficLightRoiImageSaver
{
public:
  TrafficLightRoiImageSaver();
  virtual ~TrafficLightRoiImageSaver();

private:
  void imageRoiCallback(
    const sensor_msgs::ImageConstPtr & input_image_msg,
    const autoware_perception_msgs::TrafficLightRoiArray::ConstPtr & input_tl_roi_msg);

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_perception_msgs::TrafficLightRoiArray> roi_sub_;
  image_transport::Publisher image_pub_;
  std::string save_dir_;
  std::shared_ptr<ros::Rate> save_rate_ptr_;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, autoware_perception_msgs::TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
};

}  // namespace traffic_light
