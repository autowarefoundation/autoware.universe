// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_GENERATOR__NODE_HPP_
#define AUTOWARE__PATH_GENERATOR__NODE_HPP_

#include "autoware/path_generator/path_handler.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <nav_msgs/msg/odometry.hpp>

namespace autoware::path_generator
{
class PathGenerator : public rclcpp::Node
{
public:
  explicit PathGenerator(const rclcpp::NodeOptions & node_options);

private:
  // subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, universe_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_map_msgs::msg::LaneletMapBin, universe_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>
    odometry_subscriber_{this, "~/input/odometry"};

  // publisher
  rclcpp::Publisher<tier4_planning_msgs::msg::PathWithLaneId>::SharedPtr path_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<::path_generator::ParamListener> param_listener_;

  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr_{nullptr};
  nav_msgs::msg::Odometry::ConstSharedPtr self_odometry_ptr_{nullptr};

  std::unique_ptr<PathHandler> path_handler_;

  void takeData();

  bool isDataReady();

  void run();
};
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__NODE_HPP_
