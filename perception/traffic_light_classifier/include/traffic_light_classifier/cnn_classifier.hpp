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

#include <autoware_perception_msgs/msg/lamp_state.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_light_classifier/classifier_interface.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <trt_common.hpp>

namespace traffic_light
{
class CNNClassifier : public ClassifierInterface
{
public:
  explicit CNNClassifier(rclcpp::Node * node_ptr);

  bool getLampState(
    const cv::Mat & input_image,
    std::vector<autoware_perception_msgs::msg::LampState> & states) override;

private:
  void preProcess(cv::Mat & image, std::vector<float> &  tensor, bool normalize = true);
  bool postProcess(
    std::vector<float> & output_data_host, std::vector<autoware_perception_msgs::msg::LampState> & states);
  bool readLabelfile(std::string filepath, std::vector<std::string> & labels);
  void calcSoftmax(std::vector<float> & data, std::vector<float> & probs, int num_output);
  std::vector<size_t> argsort(std::vector<float> & tensor, int num_output);
  void outputDebugImage(
    cv::Mat & debug_image, const std::vector<autoware_perception_msgs::msg::LampState> & states);

private:
  std::map<int, std::string> state2label_{
    {autoware_perception_msgs::msg::LampState::RED, "red"},
    {autoware_perception_msgs::msg::LampState::YELLOW, "yellow"},
    {autoware_perception_msgs::msg::LampState::GREEN, "green"},
    {autoware_perception_msgs::msg::LampState::UP, "straight"},
    {autoware_perception_msgs::msg::LampState::LEFT, "left"},
    {autoware_perception_msgs::msg::LampState::RIGHT, "right"},
    {autoware_perception_msgs::msg::LampState::UNKNOWN, "unknown"},
  };

  std::map<std::string, int> label2state_{
    {"red", autoware_perception_msgs::msg::LampState::RED},
    {"yellow", autoware_perception_msgs::msg::LampState::YELLOW},
    {"green", autoware_perception_msgs::msg::LampState::GREEN},
    {"straight", autoware_perception_msgs::msg::LampState::UP},
    {"left", autoware_perception_msgs::msg::LampState::LEFT},
    {"right", autoware_perception_msgs::msg::LampState::RIGHT},
    {"unknown", autoware_perception_msgs::msg::LampState::UNKNOWN},
  };

  rclcpp::Node * node_ptr_;

  std::shared_ptr<Tn::TrtCommon> trt_;
  image_transport::Publisher image_pub_;
  std::vector<std::string> labels_;
  std::vector<float> mean_{0.242, 0.193, 0.201};
  std::vector<float> std_{1.0, 1.0, 1.0};
  int input_c_;
  int input_h_;
  int input_w_;
};

}  // namespace traffic_light
