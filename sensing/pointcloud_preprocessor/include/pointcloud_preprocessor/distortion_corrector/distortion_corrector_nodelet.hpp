// Copyright 2024 Tier IV, Inc.
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

#ifndef POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODELET_HPP_

#include "pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

namespace pointcloud_preprocessor
{
using rcl_interfaces::msg::SetParametersResult;
using sensor_msgs::msg::PointCloud2;

class DistortionCorrectorComponent : public rclcpp::Node
{
public:
  explicit DistortionCorrectorComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr input_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr undistorted_points_pub_;

  std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_;

  std::string base_link_frame_ = "base_link";
  bool use_imu_;
  bool use_3d_distortion_correction_;

  std::unique_ptr<DistortionCorrectorBase> DistortionCorrector_;

  void onPointCloud(PointCloud2::UniquePtr points_msg);
  void onTwistWithCovarianceStamped(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg);
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODELET_HPP_
