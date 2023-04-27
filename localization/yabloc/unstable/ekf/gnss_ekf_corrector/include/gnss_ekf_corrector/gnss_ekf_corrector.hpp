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
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace yabloc::ekf_corrector
{
class GnssEkfCorrector : public rclcpp::Node
{
public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using NavPVT = ublox_msgs::msg::NavPVT;
  using Float32 = std_msgs::msg::Float32;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  GnssEkfCorrector();

private:
  const bool ignore_less_than_float_;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_debug_pose_;
  rclcpp::Subscription<NavPVT>::SharedPtr sub_ublox_;
  rclcpp::Subscription<Float32>::SharedPtr sub_height_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_;
  Float32 latest_height_;
  Eigen::Vector3f current_position_;

  void on_ublox(const NavPVT::ConstSharedPtr ublox_msg);
  void publish_marker(const Eigen::Vector3f & position, bool fixed);
};
}  // namespace yabloc::ekf_corrector
