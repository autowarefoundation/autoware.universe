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

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "trt_ssd.h"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include <autoware_perception_msgs/TrafficLightRoiArray.h>

typedef struct Detection
{
  float x, y, w, h, prob;
} Detection;

namespace traffic_light
{
class TrafficLightSSDFineDetector
{
public:
  TrafficLightSSDFineDetector();
  ~TrafficLightSSDFineDetector();

private:
  // functions
  void callback(
    const sensor_msgs::Image::ConstPtr & image_msg,
    const autoware_perception_msgs::TrafficLightRoiArray::ConstPtr & traffic_light_roi_msg);

  bool cvMat2CnnInput(
    const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<float> & data);
  bool cnnOutput2BoxDetection(
    const float * scores, const float * boxes, const int tlr_id,
    const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<Detection> & detctions);
  bool rosMsg2CvMat(const sensor_msgs::Image::ConstPtr & image_msg, cv::Mat & image);
  bool fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size);
  void cvRect2TlRoiMsg(
    const cv::Rect & rect, const int32_t id, autoware_perception_msgs::TrafficLightRoi & tl_roi);
  bool readLabelFile(std::string filepath, std::vector<std::string> & labels);
  bool getTlrIdFromLabel(const std::vector<std::string> & labels, int & tlr_id);

  // variables
  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_perception_msgs::TrafficLightRoiArray> roi_sub_;
  ros::Publisher output_roi_pub_;
  ros::Publisher exe_time_pub_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, autoware_perception_msgs::TrafficLightRoiArray>
    SyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, autoware_perception_msgs::TrafficLightRoiArray>
    ApproximateSyncPolicy;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;

  bool is_approximate_sync_;
  double score_thresh_;

  int tlr_id_;
  int channel_;
  int width_;
  int height_;
  int class_num_;
  int detection_per_class_;

  std::vector<float> mean_;
  std::vector<float> std_;

  std::unique_ptr<ssd::Net> net_ptr_;

};  // TrafficLightSSDFineDetector

}  // namespace traffic_light
