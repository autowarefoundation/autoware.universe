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

#include <autoware_perception_msgs/LampState.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <traffic_light_classifier/HSVFilterConfig.h>
#include <traffic_light_classifier/classifier_interface.hpp>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <trt_common.h>

namespace traffic_light
{
class CNNClassifier : public ClassifierInterface
{
public:
  CNNClassifier();
  ~CNNClassifier(){};

  bool getLampState(
    const cv::Mat & input_image,
    std::vector<autoware_perception_msgs::LampState> & states) override;

private:
  void parametersCallback(traffic_light_classifier::HSVFilterConfig & config, uint32_t level);
  void preProcess(cv::Mat & image, float * tensor, bool normalize = true);
  bool postProcess(
    float * output_data_host, std::vector<autoware_perception_msgs::LampState> & states);
  bool readLabelfile(std::string filepath, std::vector<std::string> & labels);
  void calcSoftmax(float * data, std::vector<float> & probs, int num_output);
  std::vector<size_t> argsort(float * tensor, int num_output);
  void outputDebugImage(
    cv::Mat & debug_image, const std::vector<autoware_perception_msgs::LampState> & states);

private:
  std::map<int, std::string> state2label_{
    {autoware_perception_msgs::LampState::RED, "stop"},
    {autoware_perception_msgs::LampState::YELLOW, "warning"},
    {autoware_perception_msgs::LampState::GREEN, "go"},
    {autoware_perception_msgs::LampState::UNKNOWN, "unknown"},
  };
  std::shared_ptr<Tn::TrtCommon> trt_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher image_pub_;
  std::vector<std::string> labels_;
  std::vector<float> mean_{0.242, 0.193, 0.201};
  std::vector<float> std_{1.0, 1.0, 1.0};
  int input_c_;
  int input_h_;
  int input_w_;
};

}  // namespace traffic_light
