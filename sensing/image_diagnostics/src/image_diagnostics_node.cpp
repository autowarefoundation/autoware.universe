// Copyright 2022 TIER IV, Inc.
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

#include "image_diagnostics/image_diagnostics_node.hpp"

#include <std_msgs/msg/header.hpp>

namespace image_diagnostics
{
using image_diagnostics::Image_State;
ImageDiagNode::ImageDiagNode(const rclcpp::NodeOptions & node_options)
: Node("image_diagnostics_node", node_options)
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "input/raw_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageDiagNode::ImageChecker, this, std::placeholders::_1));
  block_diag_image_pub_ =
    image_transport::create_publisher(this, "image_diag/debug/diag_block_image");
  dft_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/dft_image");
  gray_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/gray_image");

  image_state_pub_ = create_publisher<tier4_debug_msgs::msg::Int32Stamped>(
    "image_diag/image_state_diag", rclcpp::SensorDataQoS());

  updater_.setHardwareID("Image_Diagnostics");
  updater_.add(
    std::string(this->get_namespace()) + ": Image_Diagnostics", this,
    &ImageDiagNode::onImageDiagChecker);
  updater_.setPeriod(0.1);
}

void ImageDiagNode::onImageDiagChecker(DiagnosticStatusWrapper & stat)
{
  stat.add("number dark blocks ", std::to_string(num_of_regions_dark));
  stat.add("number blockaged blocks ", std::to_string(num_of_regions_blockage));
  stat.add("number low visibility blocks ", std::to_string(num_of_regions_low_visibility));
  stat.add("number backlight  blocks ", std::to_string(num_of_regions_backlight));

  auto level = DiagnosticStatusWrapper::OK;
  if (diagnostic_status_ < 0) {
    level = DiagnosticStatusWrapper::STALE;
  } else if (diagnostic_status_ == 1) {
    level = DiagnosticStatusWrapper::WARN;
  } else if (diagnostic_status_ == 2) {
    level = DiagnosticStatusWrapper::ERROR;
  } else {
    level = DiagnosticStatusWrapper::OK;
  }

  std::string msg;
  if (level == DiagnosticStatusWrapper::OK) {
    msg = "OK";
  } else if (level == DiagnosticStatusWrapper::WARN) {
    msg = "WARNING: abnormal state in image diagnostics";
  } else if (level == DiagnosticStatusWrapper::ERROR) {
    msg = "ERROR: abnormal state in image diagnostics";
  } else if (level == DiagnosticStatusWrapper::STALE) {
    msg = "STALE";
  }
  stat.summary(level, msg);
}

void ImageDiagNode::ImageChecker(const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg)
{
  cv::Mat img_gray;
  cv::Mat img_gray_32b;
  cv::Mat img_gray_blockage_bin;
  cv::Mat tmp;

  img_gray = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  cv::Size size;
  size.height = image_resize_height_;
  size.width = static_cast<int>((img_gray.cols * size.height) / img_gray.rows);
  cv::resize(img_gray, img_gray, size);
  int block_size_h = std::floor(img_gray.cols / number_block_horizontal_);
  int block_size_v = std::floor(img_gray.rows / number_block_vertical_);
  int region_pix_count = block_size_h * block_size_v;

  int block_size_h_dft = cv::getOptimalDFTSize(block_size_h);
  int block_size_v_dft = cv::getOptimalDFTSize(block_size_v);

  std::vector<int> region_average_vec;
  std::vector<float> region_blockage_ratio_vec;
  std::vector<float> region_freq_sum_vec;
  cv::threshold(
    img_gray, img_gray_blockage_bin, blockage_intensity_thresh_, 255, cv::THRESH_BINARY);
  img_gray.convertTo(img_gray_32b, CV_32FC1);
  cv::Mat imgDCT(size, CV_32FC1);
  imgDCT.setTo(0.0);
  float freqSum = 0.0;
  cv::Mat imgDFT(size, CV_32FC1);
  imgDFT.setTo(0.0);
  for (int v = 0; v < number_block_vertical_; v++) {
    for (int h = 0; h < number_block_horizontal_; h++) {
      int x = h * block_size_h;
      int y = v * block_size_v;
      cv::Rect roi(x, y, block_size_h, block_size_v);
      int intensity_average = static_cast<int>(cv::mean(img_gray(roi))[0]);
      int blockage_pix_num = cv::countNonZero(img_gray_blockage_bin(roi));
      float roi_blockage_ratio =
        (static_cast<float>(region_pix_count) - static_cast<float>(blockage_pix_num)) /
        static_cast<float>(region_pix_count);
      cv::copyMakeBorder(
        img_gray_32b(roi), img_gray_32b(roi), 0, block_size_v_dft - img_gray_32b(roi).rows, 0,
        block_size_h_dft - img_gray_32b(roi).cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
      std::vector<cv::Mat> channelImg{
        img_gray_32b(roi), cv::Mat::zeros(img_gray_32b(roi).size(), CV_32FC1)};
      cv::Mat srcComp, freqComp;
      cv::merge(channelImg, srcComp);
      cv::dft(srcComp, freqComp);
      shiftImage(freqComp);
      cv::split(freqComp, channelImg);
      cv::Rect original_roi(0, 0, block_size_h, block_size_v);
      channelImg[0](original_roi)
        .copyTo(imgDFT(cv::Rect(roi.x, roi.y, block_size_h, block_size_v)));

      cv::Mat recttmp = img_gray_32b(roi);
      channelImg[0](original_roi).copyTo(recttmp);
      cv::log(recttmp, recttmp);
      freqSum = cv::mean(recttmp)[0];

      region_average_vec.push_back(intensity_average);
      region_blockage_ratio_vec.push_back(roi_blockage_ratio);
      region_freq_sum_vec.push_back(freqSum);
    }
  }

  std::vector<int> region_state_vec;
  int region_state;

  for (int region = 0; region < number_block_horizontal_ * number_block_vertical_; region += 1) {
    if (region_average_vec[region] < dark_intensity_thresh_) {
      region_state = Image_State::DARK;
    } else if (
      region_blockage_ratio_vec[region] > blockage_ratio_thresh_ / 100 &&
      region_freq_sum_vec[region] < blockage_freq_ratio_thresh_ / 100) {
      region_state = Image_State::BLOCKAGE;
    } else if (
      region_freq_sum_vec[region] < lowVis_freq_thresh_ / 100 &&
      region_average_vec[region] < backlight_intensity_thresh_) {
      region_state = Image_State::LOW_VIS;
    } else if (region_average_vec[region] > backlight_intensity_thresh_) {
      region_state = Image_State::BACKLIGHT;
    } else {
      region_state = Image_State::NORMAL;
    }
    region_state_vec.push_back(region_state);
  }

  cv::Mat diag_block_image(size, CV_8UC3);
  int j = 0;
  for (int v = 0; v < number_block_vertical_; v++) {
    for (int h = 0; h < number_block_horizontal_; h++) {
      int x = h * block_size_h;
      int y = v * block_size_v;
      if (region_state_vec[j] == Image_State::DARK) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          state_color_map_["DARK"], -1, cv::LINE_AA);
      } else if (region_state_vec[j] == Image_State::BLOCKAGE) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          state_color_map_["BLOCKAGE"], -1, cv::LINE_AA);
      } else if (region_state_vec[j] == Image_State::LOW_VIS) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          state_color_map_["BACKLIGHT"], -1, cv::LINE_AA);
      } else if (region_state_vec[j] == Image_State::BACKLIGHT) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          state_color_map_["BACKLIGHT"], -1, cv::LINE_AA);
      } else if (region_state_vec[j] == Image_State::NORMAL) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          state_color_map_["NORMAL"], -1, cv::LINE_AA);
      }
      j = j + 1;
    }
  }

  for (int v = 1; v < number_block_vertical_; v++) {
    cv::line(
      diag_block_image, cv::Point(0, v * block_size_v), cv::Point(size.width, v * block_size_v),
      state_color_map_["BORDER"], 1, cv::LINE_AA, 0);
  }
  for (int h = 1; h < number_block_horizontal_; h++) {
    cv::line(
      diag_block_image, cv::Point(h * block_size_h, 0), cv::Point(h * block_size_h, size.height),
      state_color_map_["BORDER"], 1, cv::LINE_AA, 0);
  }

  num_of_regions_normal =
    std::count(region_state_vec.begin(), region_state_vec.end(), Image_State::NORMAL);
  num_of_regions_dark =
    std::count(region_state_vec.begin(), region_state_vec.end(), Image_State::DARK);
  num_of_regions_blockage =
    std::count(region_state_vec.begin(), region_state_vec.end(), Image_State::BLOCKAGE);
  num_of_regions_low_visibility =
    std::count(region_state_vec.begin(), region_state_vec.end(), Image_State::LOW_VIS);
  num_of_regions_backlight =
    std::count(region_state_vec.begin(), region_state_vec.end(), Image_State::BACKLIGHT);

  if (
    (num_of_regions_dark > dark_regions_num_error_thresh_) ||
    (num_of_regions_blockage > blockage_region_num_error_thresh_) ||
    (num_of_regions_low_visibility > lowVis_region_num_error_thresh_) ||
    (num_of_regions_backlight > backlight_region_num_error_thresh)) {
    diagnostic_status_ = 2;
  } else if (
    (num_of_regions_dark > dark_regions_num_warn_thresh_) ||
    (num_of_regions_blockage > blockage_region_num_warn_thresh_) ||
    (num_of_regions_low_visibility > lowVis_region_num_warn_thresh_) ||
    (num_of_regions_backlight > backlight_region_num_warn_thresh)) {
    diagnostic_status_ = 1;
  } else {
    diagnostic_status_ = 0;
  }
  tier4_debug_msgs::msg::Int32Stamped image_state_out;
  image_state_out.data = diagnostic_status_;
  image_state_pub_->publish(image_state_out);

  img_gray.convertTo(img_gray, CV_8UC1);
  imgDFT.convertTo(imgDFT, CV_8UC1);

  sensor_msgs::msg::Image::SharedPtr gray_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img_gray).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr dft_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imgDFT).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr block_diag_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", diag_block_image).toImageMsg();

  gray_image_msg->header = input_image_msg->header;
  dft_image_msg->header = input_image_msg->header;
  block_diag_image_msg->header = input_image_msg->header;

  gray_image_pub_.publish(gray_image_msg);
  dft_image_pub_.publish(dft_image_msg);
  block_diag_image_pub_.publish(block_diag_image_msg);
}

void ImageDiagNode::shiftImage(cv::Mat & img)
{
  int cx = img.cols / 2, cy = img.rows / 2;
  cv::Mat left_top(img, cv::Rect(0, 0, cx, cy)), right_top(img, cv::Rect(cx, 0, cx, cy));
  cv::Mat left_bottom(img, cv::Rect(0, cy, cx, cy)), right_bottom(img, cv::Rect(cx, cy, cx, cy)),
    tmp;
  left_top.copyTo(tmp);
  right_bottom.copyTo(left_top);
  tmp.copyTo(right_bottom);

  right_top.copyTo(tmp);
  left_bottom.copyTo(right_top);
  tmp.copyTo(left_bottom);
}

}  // namespace image_diagnostics

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_diagnostics::ImageDiagNode)
