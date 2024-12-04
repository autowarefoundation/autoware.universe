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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware/path_generator/common_structs.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <path_generator_parameters.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
namespace autoware::path_generator
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using nav_msgs::msg::Odometry;
using ::path_generator::Params;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;
using Trajectory = autoware::trajectory::Trajectory<PathPointWithLaneId>;

class PathGenerator : public rclcpp::Node
{
public:
  explicit PathGenerator(const rclcpp::NodeOptions & node_options);

private:
  struct InputData
  {
    LaneletRoute::ConstSharedPtr route_ptr{nullptr};
    LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr{nullptr};
    Odometry::ConstSharedPtr odometry_ptr{nullptr};
  };

  // subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<
    LaneletRoute, universe_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<
    LaneletMapBin, universe_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> odometry_subscriber_{
    this, "~/input/odometry"};

  // publisher
  rclcpp::Publisher<PathWithLaneId>::SharedPtr path_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<::path_generator::ParamListener> param_listener_;

  PlannerData planner_data_;

  void run();

  InputData take_data();

  void set_planner_data(const InputData & input_data);

  void set_route(const LaneletRoute::ConstSharedPtr & route_ptr);

  bool is_data_ready(const InputData & input_data);

  std::optional<PathWithLaneId> plan_path(const InputData & input_data);

  std::optional<PathWithLaneId> generate_path(
    const geometry_msgs::msg::Pose & current_pose, const Params & params) const;

  std::optional<PathWithLaneId> generate_path(
    const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & current_pose,
    const Params & params) const;

  std::optional<PathWithLaneId> generate_path(
    const lanelet::ConstLanelets & lanelets, const double s_start, const double s_end,
    const Params & params) const;
};
}  // namespace autoware::path_generator

#endif  // NODE_HPP_
