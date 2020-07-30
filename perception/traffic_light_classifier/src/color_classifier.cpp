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
#include "traffic_light_classifier/color_classifier.hpp"

#include <opencv2/imgproc/imgproc_c.h>

namespace traffic_light
{
ColorClassifier::ColorClassifier(const ros::NodeHandle & nh, const ros::NodeHandle & pnh)
: nh_(nh), pnh_(pnh), image_transport_(pnh), ratio_threshold_(0.02)
{
  image_transport_ = image_transport::ImageTransport(pnh_);
  image_pub_ = image_transport_.advertise("debug/image", 1);
  dynamic_reconfigure_.setCallback(boost::bind(&ColorClassifier::parametersCallback, this, _1, _2));
}

bool ColorClassifier::getLampState(
  const cv::Mat & input_image, std::vector<autoware_perception_msgs::LampState> & states)
{
  cv::Mat green_image;
  cv::Mat yellow_image;
  cv::Mat red_image;
  filterHSV(input_image, green_image, yellow_image, red_image);
  // binalize
  cv::Mat green_bin_image;
  cv::Mat yellow_bin_image;
  cv::Mat red_bin_image;
  const int bin_threshold = 127;
  cv::threshold(green_image, green_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
  cv::threshold(yellow_image, yellow_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
  cv::threshold(red_image, red_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
  // filter noise
  cv::Mat green_filtered_bin_image;
  cv::Mat yellow_filtered_bin_image;
  cv::Mat red_filtered_bin_image;
  cv::Mat element4 = (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
  cv::erode(green_bin_image, green_filtered_bin_image, element4, cv::Point(-1, -1), 1);
  cv::erode(yellow_bin_image, yellow_filtered_bin_image, element4, cv::Point(-1, -1), 1);
  cv::erode(red_bin_image, red_filtered_bin_image, element4, cv::Point(-1, -1), 1);
  cv::dilate(green_filtered_bin_image, green_filtered_bin_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(yellow_filtered_bin_image, yellow_filtered_bin_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(red_filtered_bin_image, red_filtered_bin_image, cv::Mat(), cv::Point(-1, -1), 1);

  /* debug */
#if 1
  if (0 < image_pub_.getNumSubscribers()) {
    cv::Mat debug_raw_image;
    cv::Mat debug_green_image;
    cv::Mat debug_yellow_image;
    cv::Mat debug_red_image;
    cv::hconcat(input_image, input_image, debug_raw_image);
    cv::hconcat(debug_raw_image, input_image, debug_raw_image);
    cv::hconcat(green_image, green_bin_image, debug_green_image);
    cv::hconcat(debug_green_image, green_filtered_bin_image, debug_green_image);
    cv::hconcat(yellow_image, yellow_bin_image, debug_yellow_image);
    cv::hconcat(debug_yellow_image, yellow_filtered_bin_image, debug_yellow_image);
    cv::hconcat(red_image, red_bin_image, debug_red_image);
    cv::hconcat(debug_red_image, red_filtered_bin_image, debug_red_image);

    cv::Mat debug_image;
    cv::vconcat(debug_green_image, debug_yellow_image, debug_image);
    cv::vconcat(debug_image, debug_red_image, debug_image);
    cv::cvtColor(debug_image, debug_image, cv::COLOR_GRAY2RGB);
    cv::vconcat(debug_raw_image, debug_image, debug_image);
    const int width = input_image.cols;
    const int height = input_image.rows;
    cv::line(
      debug_image, cv::Point(0, 0), cv::Point(debug_image.cols, 0), cv::Scalar(255, 255, 255), 1,
      CV_AA, 0);
    cv::line(
      debug_image, cv::Point(0, height), cv::Point(debug_image.cols, height),
      cv::Scalar(255, 255, 255), 1, CV_AA, 0);
    cv::line(
      debug_image, cv::Point(0, height * 2), cv::Point(debug_image.cols, height * 2),
      cv::Scalar(255, 255, 255), 1, CV_AA, 0);
    cv::line(
      debug_image, cv::Point(0, height * 3), cv::Point(debug_image.cols, height * 3),
      cv::Scalar(255, 255, 255), 1, CV_AA, 0);

    cv::line(
      debug_image, cv::Point(0, 0), cv::Point(0, debug_image.rows), cv::Scalar(255, 255, 255), 1,
      CV_AA, 0);
    cv::line(
      debug_image, cv::Point(width, 0), cv::Point(width, debug_image.rows),
      cv::Scalar(255, 255, 255), 1, CV_AA, 0);
    cv::line(
      debug_image, cv::Point(width * 2, 0), cv::Point(width * 2, debug_image.rows),
      cv::Scalar(255, 255, 255), 1, CV_AA, 0);
    cv::line(
      debug_image, cv::Point(width * 3, 0), cv::Point(width * 3, debug_image.rows),
      cv::Scalar(255, 255, 255), 1, CV_AA, 0);

    cv::putText(
      debug_image, "green", cv::Point(0, height * 1.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      cv::Scalar(255, 255, 255), 1, CV_AA);
    cv::putText(
      debug_image, "yellow", cv::Point(0, height * 2.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      cv::Scalar(255, 255, 255), 1, CV_AA);
    cv::putText(
      debug_image, "red", cv::Point(0, height * 3.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      cv::Scalar(255, 255, 255), 1, CV_AA);
    sensor_msgs::ImagePtr debug_image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
    image_pub_.publish(debug_image_msg);
  }
#endif
  /* --- */

  const int green_pixel_num = cv::countNonZero(green_filtered_bin_image);
  const int yellow_pixel_num = cv::countNonZero(yellow_filtered_bin_image);
  const int red_pixel_num = cv::countNonZero(red_filtered_bin_image);
  const double green_ratio =
    (double)green_pixel_num /
    (double)(green_filtered_bin_image.rows * green_filtered_bin_image.cols);
  const double yellow_ratio =
    (double)yellow_pixel_num /
    (double)(yellow_filtered_bin_image.rows * yellow_filtered_bin_image.cols);
  const double red_ratio =
    (double)red_pixel_num / (double)(red_filtered_bin_image.rows * red_filtered_bin_image.cols);

  if (yellow_ratio < green_ratio && red_ratio < green_ratio) {
    autoware_perception_msgs::LampState state;
    state.type = autoware_perception_msgs::LampState::GREEN;
    state.confidence = std::min(1.0, double(green_pixel_num) / (20.0 * 20.0));
    states.push_back(state);
  } else if (green_ratio < yellow_ratio && red_ratio < yellow_ratio) {
    autoware_perception_msgs::LampState state;
    state.type = autoware_perception_msgs::LampState::YELLOW;
    state.confidence = std::min(1.0, double(yellow_pixel_num) / (20.0 * 20.0));
    states.push_back(state);
  } else if (green_ratio < red_ratio && yellow_ratio < red_ratio) {
    autoware_perception_msgs::LampState state;
    state.type = autoware_perception_msgs::LampState::RED;
    state.confidence = std::min(1.0, double(red_pixel_num) / (20.0 * 20.0));
    states.push_back(state);
  } else {
    autoware_perception_msgs::LampState state;
    state.type = autoware_perception_msgs::LampState::UNKNOWN;
    state.confidence = 0.0;
    states.push_back(state);
  }
  return true;
}

bool ColorClassifier::filterHSV(
  const cv::Mat & input_image, cv::Mat & green_image, cv::Mat & yellow_image, cv::Mat & red_image)
{
  cv::Mat hsv_image;
  cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
  try {
    cv::inRange(hsv_image, min_hsv_green_, max_hsv_green_, green_image);
    cv::inRange(hsv_image, min_hsv_yellow_, max_hsv_yellow_, yellow_image);
    cv::inRange(hsv_image, min_hsv_red_, max_hsv_red_, red_image);
  } catch (cv::Exception & e) {
    ROS_ERROR("failed to filter image by hsv value : %s", e.what());
    return false;
  }
  return true;
}
void ColorClassifier::parametersCallback(
  traffic_light_classifier::HSVFilterConfig & config, uint32_t level)
{
  min_hsv_green_ = cv::Scalar(config.green_min_h, config.green_min_s, config.green_min_v);
  max_hsv_green_ = cv::Scalar(config.green_max_h, config.green_max_s, config.green_max_v);
  min_hsv_yellow_ = cv::Scalar(config.yellow_min_h, config.yellow_min_s, config.yellow_min_v);
  max_hsv_yellow_ = cv::Scalar(config.yellow_max_h, config.yellow_max_s, config.yellow_max_v);
  min_hsv_red_ = cv::Scalar(config.red_min_h, config.red_min_s, config.red_min_v);
  max_hsv_red_ = cv::Scalar(config.red_max_h, config.red_max_s, config.red_max_v);
}

}  // namespace traffic_light
