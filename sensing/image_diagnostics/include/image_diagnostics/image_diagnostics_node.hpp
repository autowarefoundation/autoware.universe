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

#ifndef IMAGE_DIAGNOSTICS__IMAGE_DIAGNOSTICS_NODE_HPP_
#define IMAGE_DIAGNOSTICS__IMAGE_DIAGNOSTICS_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <unordered_map>
namespace image_diagnostics
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater; 

std::unordered_map<uint8_t, std::string> region_state_map_ = {
  {0, "Normal"}, {1, "TooDark"}, {2, "Blockage"}, {3, "Low Vis"}, {4, "Backlight"}};

class ImageDiagNode : public rclcpp::Node
{
private:
  void CompressedImageChecker(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg);
  void shiftImage(cv::Mat & img);
  void onImageDiagChecker(DiagnosticStatusWrapper & stat);
  
  int num_of_regions_normal;
  int num_of_regions_dark;
  int num_of_regions_blockage;
  int num_of_regions_low_visibility;
  int num_of_regions_blacklight;

  int image_resize_height_ = 480;
  int image_state_;
  int number_block_horizontal_ = 5;
  int number_block_vertical_ = 5;

  int NumOfRegionsDark_Warn = 10;
  int NumOfRegionsBlockage_Warn = 3;
  int NumOfRegionsLowVisibility_Warn = 2;
  int NumOfRegionsBacklight_Warn = 2;

  int NumOfRegionsDark_Error = 20;
  int NumOfRegionsBlockage_Error = 5;
  int NumOfRegionsLowVisibility_Error = 4;
  int NumOfRegionsBacklight_Error = 3;

  float BlockageRatio = 90.0f;
  int BlockagePixValue = 10;
  float BlockageFrequencyRatio = 30.0f;

  int TooDarkTh = 10;
  float VisibilityTh = 400.0f;
  int BacklightTh = 230;

  // int ImgResizeFactor = 2;
  // float Ref_freq = 800000;
  float Ref_freq = 1.0f;
  int RegionState;
  Updater updater_{this};


public:
  explicit ImageDiagNode(const rclcpp::NodeOptions & node_options);

protected: 
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  image_transport::Publisher block_diag_image_pub_;
  image_transport::Publisher dft_image_pub_;
  image_transport::Publisher gray_image_pub_;
  image_transport::Publisher raw_image_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr average_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr frequency_intensity_pub1_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Int32Stamped>::SharedPtr image_state_pub_;

};

}  // namespace image_diagnostics

#endif  // IMAGE_DIAGNOSTICS__IMAGE_DIAGNOSTICS_NODE_HPP_
