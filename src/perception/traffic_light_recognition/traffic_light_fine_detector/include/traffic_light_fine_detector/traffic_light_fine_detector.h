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
#ifndef TRAFFIC_LIGHT_DETECTOR_H
#define TRAFFIC_LIGHT_DETECTOR_H

#include <memory>
#include <string>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TrtNet.h"
#include "data_reader.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <autoware_perception_msgs/TrafficLightRoiArray.h>

typedef struct Detection
{
  float x, y, w, h, prob;
} Detection;

namespace traffic_light
{
class TrafficLightFineDetectorNode
{
private:
  // variables

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, autoware_perception_msgs::TrafficLightRoiArray>
    SyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, autoware_perception_msgs::TrafficLightRoiArray>
    ApproximateSyncPolicy;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<autoware_perception_msgs::TrafficLightRoiArray>
    traffic_light_roi_sub_;
  ros::Publisher output_rois_pub_;

  bool is_approximate_sync_;
  double score_thresh_;

  std::unique_ptr<Tn::trtNet> net_ptr_;
  std::vector<std::vector<int> > output_shape = {{1, 18, 8, 8}, {1, 18, 16, 16}, {1, 18, 32, 32}};
  std::vector<std::vector<int> > g_masks = {{6, 7, 8}, {3, 4, 5}, {0, 1, 2}};
  std::vector<std::vector<int> > g_anchors = {{72, 35},  {53, 58},  {127, 43}, {90, 62},  {61, 104},
                                              {141, 70}, {105, 95}, {95, 154}, {157, 135}};

  // functions
  void initROS();

  void callback(
    const sensor_msgs::Image::ConstPtr & image_msg,
    const autoware_perception_msgs::TrafficLightRoiArray::ConstPtr & traffic_light_roi_msg);

  inline float sigmoid(float in) { return 1.f / (1.f + std::exp(-in)); }

  std::vector<float> prepareImage(cv::Mat & in_img);
  std::vector<Tn::Bbox> postProcessImg(float * output, const int classes, cv::Mat & img);
  void doNms(std::vector<Detection> & detections, float nms_thresh);
  bool rosmsg2cvmat(const sensor_msgs::Image::ConstPtr & image_msg, cv::Mat & image);
  bool fit_in_frame(cv::Point & lt, cv::Point & rb, const cv::Size & size);
  void cvrect2tlroimsg(
    const cv::Rect & rect, const int32_t id, autoware_perception_msgs::TrafficLightRoi & tl_roi);

public:
  TrafficLightFineDetectorNode();
  ~TrafficLightFineDetectorNode();
  void run();

};  // TrafficLightFineDetector

}  // namespace traffic_light

#endif
