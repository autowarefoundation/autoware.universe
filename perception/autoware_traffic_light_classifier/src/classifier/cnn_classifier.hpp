// Copyright 2023 TIER IV, Inc.
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

#ifndef CLASSIFIER__CNN_CLASSIFIER_HPP_
#define CLASSIFIER__CNN_CLASSIFIER_HPP_

#include "classifier_interface.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/stream_unique_ptr.hpp>
#include <autoware/tensorrt_classifier/tensorrt_classifier.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

class CNNClassifier : public ClassifierInterface
{
public:
  explicit CNNClassifier(rclcpp::Node * node_ptr);
  virtual ~CNNClassifier() = default;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) override;

private:
  void postProcess(
    int class_index, float prob, tier4_perception_msgs::msg::TrafficLight & traffic_signal);
  bool readLabelfile(std::string filepath, std::vector<std::string> & labels);
  bool isColorLabel(const std::string & label);
  void outputDebugImage(
    cv::Mat & debug_image, const tier4_perception_msgs::msg::TrafficLight & traffic_signal);

private:
  std::map<int, std::string> state2label_{
    // color
    {tier4_perception_msgs::msg::TrafficLightElement::RED, "red"},
    {tier4_perception_msgs::msg::TrafficLightElement::AMBER, "yellow"},
    {tier4_perception_msgs::msg::TrafficLightElement::GREEN, "green"},
    {tier4_perception_msgs::msg::TrafficLightElement::WHITE, "white"},
    // shape
    {tier4_perception_msgs::msg::TrafficLightElement::CIRCLE, "circle"},
    {tier4_perception_msgs::msg::TrafficLightElement::LEFT_ARROW, "left"},
    {tier4_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW, "right"},
    {tier4_perception_msgs::msg::TrafficLightElement::UP_ARROW, "straight"},
    {tier4_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW, "up_left"},
    {tier4_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW, "up_right"},
    {tier4_perception_msgs::msg::TrafficLightElement::DOWN_ARROW, "down"},
    {tier4_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW, "down_left"},
    {tier4_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW, "down_right"},
    {tier4_perception_msgs::msg::TrafficLightElement::CROSS, "cross"},
    // other
    {tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN, "unknown"},
  };

  std::map<std::string, int> label2state_{
    // color
    {"red", tier4_perception_msgs::msg::TrafficLightElement::RED},
    {"yellow", tier4_perception_msgs::msg::TrafficLightElement::AMBER},
    {"green", tier4_perception_msgs::msg::TrafficLightElement::GREEN},
    {"white", tier4_perception_msgs::msg::TrafficLightElement::WHITE},
    // shape
    {"circle", tier4_perception_msgs::msg::TrafficLightElement::CIRCLE},
    {"left", tier4_perception_msgs::msg::TrafficLightElement::LEFT_ARROW},
    {"right", tier4_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW},
    {"straight", tier4_perception_msgs::msg::TrafficLightElement::UP_ARROW},
    {"up_left", tier4_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW},
    {"up_right", tier4_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW},
    {"down", tier4_perception_msgs::msg::TrafficLightElement::DOWN_ARROW},
    {"down_left", tier4_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW},
    {"down_right", tier4_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW},
    {"cross", tier4_perception_msgs::msg::TrafficLightElement::CROSS},
    // other
    {"unknown", tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN},
  };

  rclcpp::Node * node_ptr_;
  int batch_size_;
  std::unique_ptr<autoware::tensorrt_classifier::TrtClassifier> classifier_;
  image_transport::Publisher image_pub_;
  std::vector<std::string> labels_;
  std::vector<float> mean_;
  std::vector<float> std_;
};

}  // namespace autoware::traffic_light

#endif  // CLASSIFIER__CNN_CLASSIFIER_HPP_
