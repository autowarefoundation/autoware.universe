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
#include "corrector_manager/init_area.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcdless
{
class CorrectorManager : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetBool = std_srvs::srv::SetBool;

  CorrectorManager();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_init_area_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Client<SetBool>::SharedPtr client_;

  std::optional<InitArea> init_area_{std::nullopt};
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  void call_service(bool data);

  void on_timer();
  void on_init_area(const PointCloud2 & msg);
  void on_gnss_pose(const PoseStamped & msg);
};
}  // namespace pcdless