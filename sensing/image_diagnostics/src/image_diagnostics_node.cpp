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

#include <std_msgs/msg/header.hpp>￼


namespace image_diagnostics
{

ImageDiagNode::ImageDiagNode(const rclcpp::NodeOptions & node_options)
: Node("image_diagnostics_node", node_options)
{
  {
    number_block_horizontal_ = static_cast<int>(declare_parameter("number_block_horizontal",5));
    number_block_vertical_ = static_cast<int>(declare_parameter("number_block_vertical",5));
  }
  compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    "input/compressed_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageDiagNode::CompressedImageChecker, this, std::placeholders::_1));
  block_diag_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/diag_block_image");
  dft_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/dft_image");
  gray_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/gray_image");
  raw_image_pub_ = image_transport::create_publisher(this, "image_diag/raw_image");
  frequency_intensity_pub1_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "image_diag/frequencey_intensity_ratio", rclcpp::SensorDataQoS());

  image_state_pub_ = create_publisher<tier4_debug_msgs::msg::Int32Stamped>(
    "image_diag/image_state_diag", rclcpp::SensorDataQoS());

  updater_.setHardwareID("Image_Diagnostics");
  updater_.add(
    std::string(this->get_namespace()) + ": Image_Diagnostics", this,
    &ImageDiagNode::onImageDiagChecker);
  updater_.setPeriod(0.1);

}

void ImageDiagNode::onImageDiagChecker(DiagnosticStatusWrapper & stat){
  stat.add("image status", std::to_string(image_state_));

  // auto level = DiagnosticStatusWrapper
  auto level = DiagnosticStatusWrapper::OK;
  if (image_state_ < 0){
    level = DiagnosticStatusWrapper::STALE;
  } else if (image_state_ == 1){
    level = DiagnosticStatusWrapper::WARN;
  }else if (image_state_ == 2){
    level = DiagnosticStatusWrapper::ERROR;
  }
  else {
    level = DiagnosticStatusWrapper::OK;
  }

  std::string msg;
  if (level == DiagnosticStatusWrapper::OK){
    msg = "OK";
  }else if (level == DiagnosticStatusWrapper::WARN){
    msg = "WARNING: ";
  }
  stat.summary(level,msg);
}


void ImageDiagNode::CompressedImageChecker(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg)
{
  cv::Mat img_gray;
  cv::Mat img_gray_32b;
  cv::Mat img_gray_blockage_bin;
  cv::Mat tmp;
  cv_bridge::CvImagePtr cv_prt(new cv_bridge::CvImage);
  cv_prt->header = input_compressed_image_msg->header;
  cv_prt->image = cv::imdecode(cv::Mat(input_compressed_image_msg->data), cv::IMREAD_COLOR);

  sensor_msgs::msg::Image::SharedPtr raw_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_prt->image).toImageMsg();

  raw_image_msg->header = input_compressed_image_msg->header;
  raw_image_pub_.publish(raw_image_msg);
  img_gray = cv_bridge::toCvCopy(raw_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  cv::Size size;
  size.height = image_resize_height_;
  size.width = static_cast<int>((img_gray.cols * size.height) / img_gray.rows);
  cv::resize(img_gray, img_gray, size);
  int block_size_h = std::floor(img_gray.cols / number_block_horizontal_);
  int block_size_v = std::floor(img_gray.rows / number_block_vertical_);
  int region_pix_count = block_size_h * block_size_v;

  int block_size_h_dft = cv::getOptimalDFTSize(block_size_h);
  int block_size_v_dft = cv::getOptimalDFTSize(block_size_v);

  int average[25];
  int BlockagePixNum[number_block_horizontal_ * number_block_vertical_];
  int i = 0;

  std::vector<int> average_vector;
  std::vector<float> BlockageRatioVec;
  std::vector<float> FreqIntVec;
  cv::threshold(img_gray, img_gray_blockage_bin, BlockagePixValue, 255, cv::THRESH_BINARY);
  img_gray.convertTo(img_gray_32b, CV_32FC1);
  cv::Mat imgDCT(size, CV_32FC1);
  imgDCT.setTo(0.0);
  float freqSum = 0.0;
  cv::Mat imgDFT(size, CV_32FC1);
  imgDFT.setTo(0.0);
  for (int v = 0; v < number_block_vertical_; v++){
    for (int h = 0; h < number_block_horizontal_; h++){
      int x = h* block_size_h;
      int y = v* block_size_v;
      cv::Rect roi(x, y, block_size_h, block_size_v);
      average[i] = static_cast<int>(cv::mean(img_gray(roi))[0]);
      BlockagePixNum[i] = cv::countNonZero(img_gray_blockage_bin(roi));
      float BlockageRatioCal =
        (static_cast<float>(region_pix_count) - static_cast<float>(BlockagePixNum[i])) /
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
      channelImg[0](original_roi).copyTo(imgDFT(cv::Rect(roi.x, roi.y, block_size_h, block_size_v)));

      // // freqSum = cv::mean(channelImg[0])[0];

      cv::Mat recttmp = img_gray_32b(roi);
      channelImg[0](original_roi).copyTo(recttmp);
      cv::log(recttmp, recttmp);
      freqSum = cv::mean(recttmp)[0];

      average_vector.push_back(average[i]);
      BlockageRatioVec.push_back(BlockageRatioCal);
      FreqIntVec.push_back(freqSum);

      tier4_debug_msgs::msg::Float32Stamped freq_output;
      freq_output.data = freqSum;
      frequency_intensity_pub1_->publish(freq_output);
      i = i + 1;
    }
  }

  std::vector<int> RegionStateVec;
  int region_state;

  for (int region = 0; region < number_block_horizontal_ * number_block_vertical_; region += 1) {
    if (average_vector[region] < TooDarkTh) {
      region_state = 1;  // TooDark
    } else if (
      BlockageRatioVec[region] > BlockageRatio / 100 &&
      FreqIntVec[region] < BlockageFrequencyRatio / 100) {
      region_state = 2;  // Blockage
    } else if (FreqIntVec[region] < VisibilityTh / 100 && average_vector[region] < BacklightTh) {
      region_state = 3;  // LowVisibility
    } else if (average_vector[region] > BacklightTh) {
      region_state = 4;  // Backlight
    } else {
      region_state = 0;  // Normal
    }
    RegionStateVec.push_back(region_state);
  }

  cv::Mat diag_block_image(size, CV_8UC3);
  int j = 0;
  for (int v = 0; v < number_block_vertical_; v++){
    for (int h = 0; h < number_block_horizontal_; h++){
      int x = h* block_size_h;
      int y = v* block_size_v;
      if (RegionStateVec[j] == 1) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          cv::Scalar(0, 0, 0), -1, cv::LINE_AA);  // TooDark
      } else if (RegionStateVec[j] == 2) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          cv::Scalar(0, 0, 200), -1, cv::LINE_AA);  // Blockage
      } else if (RegionStateVec[j] == 3) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          cv::Scalar(200, 0, 200), -1, cv::LINE_AA);  // LowVisibility
      } else if (RegionStateVec[j] == 4) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          cv::Scalar(0, 200, 200), -1, cv::LINE_AA);  // Backlight
      } else if (RegionStateVec[j] == 0) {
        cv::rectangle(
          diag_block_image, cv::Point(x, y), cv::Point(x + block_size_h, y + block_size_v),
          cv::Scalar(100, 100, 100), -1, cv::LINE_AA);  // Normal
      }
      cv::putText(
        diag_block_image, region_state_map_[RegionStateVec[j]], cv::Point(x + 20, y + block_size_v / 2),
        cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(255, 0, 0), 1);
      j = j + 1;
    }
  }

  cv::line(
    diag_block_image, cv::Point(0, 1 * block_size_v), cv::Point(size.width, 1 * block_size_v),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);
  cv::line(
    diag_block_image, cv::Point(0, 2 * block_size_v), cv::Point(size.width, 2 * block_size_v),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);
  cv::line(
    diag_block_image, cv::Point(0, 3 * block_size_v), cv::Point(size.width, 3 * block_size_v),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);
  cv::line(
    diag_block_image, cv::Point(0, 4 * block_size_v), cv::Point(size.width, 4 * block_size_v),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);

  cv::line(
    diag_block_image, cv::Point(1 * block_size_h, 0), cv::Point(1 * block_size_h, size.height),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);
  cv::line(
    diag_block_image, cv::Point(2 * block_size_h, 0), cv::Point(2 * block_size_h, size.height),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);
  cv::line(
    diag_block_image, cv::Point(3 * block_size_h, 0), cv::Point(3 * block_size_h, size.height),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);
  cv::line(
    diag_block_image, cv::Point(4 * block_size_h, 0), cv::Point(4 * block_size_h, size.height),
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA, 0);

  num_of_regions_normal = std::count(RegionStateVec.begin(), RegionStateVec.end(), 0);
  num_of_regions_dark = std::count(RegionStateVec.begin(), RegionStateVec.end(), 1);
  num_of_regions_blockage = std::count(RegionStateVec.begin(), RegionStateVec.end(), 2);
  num_of_regions_low_visibility = std::count(RegionStateVec.begin(), RegionStateVec.end(), 3);
  num_of_regions_blacklight = std::count(RegionStateVec.begin(), RegionStateVec.end(), 4);

  if ((num_of_regions_dark > NumOfRegionsDark_Error) ||
  (num_of_regions_blockage > NumOfRegionsBlockage_Error) ||
  (num_of_regions_low_visibility > NumOfRegionsLowVisibility_Error) ||
  (num_of_regions_blacklight > NumOfRegionsBacklight_Error)){
    image_state_ = 2;
  }else if ((num_of_regions_dark > NumOfRegionsDark_Warn) ||
  (num_of_regions_blockage > NumOfRegionsBlockage_Warn) ||
  (num_of_regions_low_visibility > NumOfRegionsLowVisibility_Warn) ||
  (num_of_regions_blacklight > NumOfRegionsBacklight_Warn)){
    image_state_ = 1;
  }else{
    image_state_ = 0;
  }
  tier4_debug_msgs::msg::Int32Stamped imgae_state_out;
  imgae_state_out.data = image_state_;
  image_state_pub_->publish(imgae_state_out);

  img_gray.convertTo(img_gray, CV_8UC1);
  imgDFT.convertTo(imgDFT, CV_8UC1);

  sensor_msgs::msg::Image::SharedPtr gray_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img_gray).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr dft_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imgDFT).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr block_diag_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", diag_block_image).toImageMsg();

  gray_image_msg->header = input_compressed_image_msg->header;
  dft_image_msg->header = input_compressed_image_msg->header;
  block_diag_image_msg->header = input_compressed_image_msg->header;

  gray_image_pub_.publish(gray_image_msg);
  dft_image_pub_.publish(dft_image_msg);
  block_diag_image_pub_.publish(block_diag_image_msg);
}

void ImageDiagNode::shiftImage(cv::Mat & img)
{
  int cx = img.cols / 2, cy = img.rows / 2;
  cv::Mat left_top(img, cv::Rect(0, 0, cx, cy)), right_top(img, cv::Rect(cx, 0, cx, cy));
  cv::Mat left_bottom(img, cv::Rect(0, cy, cx, cy)), right_bottom(img, cv::Rect(cx, cy, cx, cy)), tmp;
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
