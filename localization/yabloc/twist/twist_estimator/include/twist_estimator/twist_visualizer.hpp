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
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <boost/circular_buffer.hpp>

#include <optional>

namespace yabloc::twist_visualizer
{
class TwistVisualizer : public rclcpp::Node
{
public:
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;

  TwistVisualizer();

private:
  rclcpp::Publisher<Path>::SharedPtr pub_path_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_stamped_;

  boost::circular_buffer<Sophus::SE3f> odoms_;

  std::optional<rclcpp::Time> last_twist_stamp_{std::nullopt};
  Sophus::SE3f last_odom_;

  void on_twist_stamped(const TwistStamped & msg);

  void publish_path(const rclcpp::Time & stmap);

  Pose se3f_to_pose_msg(const Sophus::SE3f & pose);
};
}  // namespace yabloc::twist_visualizer