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

#include <traffic_light_roi_visualizer/node.hpp>

namespace traffic_light
{
TrafficLightRoiVisualizer::TrafficLightRoiVisualizer()
: nh_(""),
  pnh_("~"),
  image_transport_(pnh_),
  image_sub_(image_transport_, "input/image", 1),
  roi_sub_(pnh_, "input/rois", 1),
  rough_roi_sub_(pnh_, "input/rough/rois", 1)
{
  pnh_.param<bool>("enable_fine_detection", enable_fine_detection_, false);

  image_pub_ = image_transport_.advertise("output/image", 1);

  if (enable_fine_detection_) {
    sync_with_rough_roi_ = boost::make_shared<message_filters::Synchronizer<SyncPolicyWithRoughRoi>>(10);
    sync_with_rough_roi_->connectInput(image_sub_, roi_sub_, rough_roi_sub_);
    sync_with_rough_roi_->registerCallback(boost::bind(&TrafficLightRoiVisualizer::imageRoughRoiCallback, this, _1, _2, _3));
  } else {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy>>(10);
    sync_->connectInput(image_sub_, roi_sub_);
    sync_->registerCallback(boost::bind(&TrafficLightRoiVisualizer::imageRoiCallback, this, _1, _2));
  }
  // cv::namedWindow("view");
}
TrafficLightRoiVisualizer::~TrafficLightRoiVisualizer()
{
  // cv::destroyWindow("view");
}

bool TrafficLightRoiVisualizer::createRect(
  cv::Mat& image,
  const autoware_perception_msgs::TrafficLightRoi& tl_roi,
  const cv::Scalar& color)
{
  cv::rectangle(
    image,
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 1, CV_AA, 0);
  cv::putText(
    image,
    std::to_string(tl_roi.id),
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 1, CV_AA);
  return true;
}

void TrafficLightRoiVisualizer::imageRoiCallback(
  const sensor_msgs::ImageConstPtr & input_image_msg,
  const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_tl_roi_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    for (auto tl_roi : input_tl_roi_msg->rois) {
      createRect(cv_ptr->image, tl_roi, cv::Scalar(0,255,0));
    }
  } catch (cv_bridge::Exception & e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}

void TrafficLightRoiVisualizer::imageRoughRoiCallback(
  const sensor_msgs::ImageConstPtr & input_image_msg,
  const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_tl_roi_msg,
  const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_tl_rough_roi_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    for (auto tl_roi : input_tl_roi_msg->rois) {
      createRect(cv_ptr->image, tl_roi, cv::Scalar(0,0,255));
    }
    for (auto tl_rough_roi : input_tl_rough_roi_msg->rois) {
      createRect(cv_ptr->image, tl_rough_roi, cv::Scalar(0,255,0));
    }
  } catch (cv_bridge::Exception & e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}

}  // namespace traffic_light
