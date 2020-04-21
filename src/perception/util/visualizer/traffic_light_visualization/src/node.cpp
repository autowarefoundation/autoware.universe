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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <traffic_light_roi_visualizer/node.hpp>

namespace traffic_light
{
TrafficLightRoiVisualizer::TrafficLightRoiVisualizer()
: nh_(""),
  pnh_("~"),
  image_transport_(pnh_),
  image_sub_(image_transport_, "input/image", 1),
  roi_sub_(pnh_, "input/rois", 1),
  sync_(SyncPolicy(10), image_sub_, roi_sub_)
{
  image_pub_ = image_transport_.advertise("output/image", 1);
  sync_.registerCallback(boost::bind(&TrafficLightRoiVisualizer::imageRoiCallback, this, _1, _2));
  // cv::namedWindow("view");
}
TrafficLightRoiVisualizer::~TrafficLightRoiVisualizer()
{
  // cv::destroyWindow("view");
}
void TrafficLightRoiVisualizer::imageRoiCallback(
  const sensor_msgs::ImageConstPtr & input_image_msg,
  const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_tl_roi_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    for (size_t i = 0; i < input_tl_roi_msg->rois.size(); ++i) {
      cv::rectangle(
        cv_ptr->image,
        cv::Point(
          input_tl_roi_msg->rois.at(i).roi.x_offset, input_tl_roi_msg->rois.at(i).roi.y_offset),
        cv::Point(
          input_tl_roi_msg->rois.at(i).roi.x_offset + input_tl_roi_msg->rois.at(i).roi.width,
          input_tl_roi_msg->rois.at(i).roi.y_offset + input_tl_roi_msg->rois.at(i).roi.height),
        cv::Scalar(0, 0, 255), 1, CV_AA, 0);
      cv::putText(
        cv_ptr->image, std::to_string(input_tl_roi_msg->rois.at(i).id),
        cv::Point(
          input_tl_roi_msg->rois.at(i).roi.x_offset, input_tl_roi_msg->rois.at(i).roi.y_offset),
        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 200), 1, CV_AA);
    }
    // cv::imshow("view", cv_ptr->image);
    // cv::waitKey(10);
  } catch (cv_bridge::Exception & e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}
}  // namespace traffic_light