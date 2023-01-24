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

#ifndef COLLISION_FREE_PATH_PLANNER__COMMON_STRUCTS_HPP_
#define COLLISION_FREE_PATH_PLANNER__COMMON_STRUCTS_HPP_

#include "collision_free_path_planner/type_alias.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace collision_free_path_planner
{
struct ReferencePoint;
struct Bounds;

struct PlannerData
{
  // input
  Header header;
  std::vector<TrajectoryPoint> traj_points;
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;

  // ego
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel{};
};

struct TimeKeeper
{
  template <typename T>
  TimeKeeper & operator<<(const T & msg)
  {
    sstream << msg;
    return *this;
  }

  void endLine()
  {
    const auto msg = sstream.str();
    accumulated_msg += msg + "\n";

    if (enable_calculation_time_info) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("collision_free_path_planner.time"), msg);
    }
    sstream.str("");
  }

  bool enable_calculation_time_info;
  std::string accumulated_msg = "\n";
  std::stringstream sstream;

  void tic(const std::string & func_name) { stop_watch_.tic(func_name); }

  void toc(const std::string & func_name, const std::string & white_spaces)
  {
    const double elapsed_time = stop_watch_.toc(func_name);
    *this << white_spaces << func_name << ":= " << elapsed_time << " [ms]";
    endLine();
  }

  std::string getAccumulatedTimeString() const { return accumulated_msg; }

  tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;
};

struct DebugData
{
  // settting
  size_t mpt_visualize_sampling_num;
  geometry_msgs::msg::Pose ego_pose;
  std::vector<double> vehicle_circle_radiuses;
  std::vector<double> vehicle_circle_longitudinal_offsets;

  // mpt
  std::vector<ReferencePoint> ref_points;
  std::vector<std::vector<geometry_msgs::msg::Pose>> vehicle_circles_pose;

  std::vector<TrajectoryPoint> extended_traj_points;
};

struct TrajectoryParam
{
  TrajectoryParam() = default;
  TrajectoryParam(rclcpp::Node * node, const double vehicle_width)
  {
    output_backward_traj_length =
      node->declare_parameter<double>("common.output_backward_traj_length");
    output_delta_arc_length = node->declare_parameter<double>("common.output_delta_arc_length");

    delta_dist_threshold_for_closest_point =
      node->declare_parameter<double>("common.delta_dist_threshold_for_closest_point");
    delta_yaw_threshold_for_closest_point =
      node->declare_parameter<double>("common.delta_yaw_threshold_for_closest_point");
    delta_yaw_threshold_for_straight =
      node->declare_parameter<double>("common.delta_yaw_threshold_for_straight");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    using tier4_autoware_utils::updateParam;

    // common
    updateParam<double>(
      parameters, "common.output_backward_traj_length", output_backward_traj_length);
    updateParam<double>(parameters, "common.output_delta_arc_length", output_delta_arc_length);

    updateParam<double>(
      parameters, "common.delta_dist_threshold_for_closest_point",
      delta_dist_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_closest_point",
      delta_yaw_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_straight", delta_yaw_threshold_for_straight);
  }

  double output_delta_arc_length;
  double output_backward_traj_length;

  double delta_dist_threshold_for_closest_point;
  double delta_yaw_threshold_for_closest_point;
  double delta_yaw_threshold_for_straight;
};

struct EgoNearestParam
{
  EgoNearestParam() = default;
  explicit EgoNearestParam(rclcpp::Node * node)
  {
    dist_threshold = node->declare_parameter<double>("ego_nearest_dist_threshold");
    yaw_threshold = node->declare_parameter<double>("ego_nearest_yaw_threshold");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    using tier4_autoware_utils::updateParam;
    updateParam<double>(parameters, "ego_nearest_dist_threshold", dist_threshold);
    updateParam<double>(parameters, "ego_nearest_yaw_threshold", yaw_threshold);
  }

  double dist_threshold{0.0};
  double yaw_threshold{0.0};
};
}  // namespace collision_free_path_planner

#endif  // COLLISION_FREE_PATH_PLANNER__COMMON_STRUCTS_HPP_
