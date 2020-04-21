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
#include "node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace traffic_light
{
TrafficLightRoiImageSaver::TrafficLightRoiImageSaver()
: nh_(""),
  pnh_("~"),
  image_transport_(pnh_),
  image_sub_(image_transport_, "input/image", 1),
  roi_sub_(pnh_, "input/rois", 1),
  sync_(SyncPolicy(10), image_sub_, roi_sub_)
{
  sync_.registerCallback(boost::bind(&TrafficLightRoiImageSaver::imageRoiCallback, this, _1, _2));

  pnh_.getParam("save_dir", save_dir_);
  double save_rate;
  pnh_.param("save_rate", save_rate, 1.0);
  save_rate_ptr_ = std::make_shared<ros::Rate>(save_rate);
}
TrafficLightRoiImageSaver::~TrafficLightRoiImageSaver() {}
void TrafficLightRoiImageSaver::imageRoiCallback(
  const sensor_msgs::ImageConstPtr & input_image_msg,
  const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_tl_roi_msg)
{
  ros::Time current_time = ros::Time::now();
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    for (size_t i = 0; i < input_tl_roi_msg->rois.size(); ++i) {
      const sensor_msgs::RegionOfInterest & roi = input_tl_roi_msg->rois.at(i).roi;
      cv::Mat cliped_image(
        cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
      std::stringstream save_fine_name_stream;
      save_fine_name_stream << std::fixed << save_dir_ << "/" << input_tl_roi_msg->rois.at(i).id
                            << "_" << current_time.toSec() << ".png";
      std::string save_fine_name;
      save_fine_name_stream >> save_fine_name;
      cv::imwrite(save_fine_name, cliped_image);
      ROS_INFO("%s", save_fine_name.c_str());
    }
  } catch (cv_bridge::Exception & e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  save_rate_ptr_->sleep();
}
}  // namespace traffic_light