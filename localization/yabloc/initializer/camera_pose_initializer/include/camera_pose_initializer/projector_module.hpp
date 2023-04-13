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

#pragma once
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcdless_common/camera_info_subscriber.hpp>
#include <pcdless_common/static_tf_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace pcdless::initializer
{
class ProjectorModule
{
public:
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const cv::Point2i &)>;
  ProjectorModule(rclcpp::Node * node);

  bool define_project_func();

  cv::Mat project_image(const sensor_msgs::msg::Image & image_msg);

private:
  ProjectFunc project_func_ = nullptr;
  rclcpp::Logger logger_;
  common::CameraInfoSubscriber info_;
  common::StaticTfSubscriber tf_subscriber_;
};
}  // namespace pcdless::initializer