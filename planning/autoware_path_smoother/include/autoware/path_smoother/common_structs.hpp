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

#ifndef AUTOWARE__PATH_SMOOTHER__COMMON_STRUCTS_HPP_
#define AUTOWARE__PATH_SMOOTHER__COMMON_STRUCTS_HPP_

#include "autoware/path_smoother/type_alias.hpp"
#include "autoware_utils/ros/update_param.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::path_smoother
{
struct Bounds;

struct PlannerData
{
  std::vector<TrajectoryPoint> traj_points;
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel{};

  PlannerData(
    std::vector<TrajectoryPoint> traj_points_, geometry_msgs::msg::Pose ego_pose_, double ego_vel_)
  : traj_points(traj_points_), ego_pose(ego_pose_), ego_vel(ego_vel_)
  {
  }
};

struct TimeKeeper
{
  void init()
  {
    accumulated_msg = "\n";
    accumulated_time = 0.0;
  }

  template <typename T>
  TimeKeeper & operator<<(const T & msg)
  {
    latest_stream << msg;
    return *this;
  }

  void endLine()
  {
    const auto msg = latest_stream.str();
    accumulated_msg += msg + "\n";
    latest_stream.str("");
  }

  void tic(const std::string & func_name) { stop_watch_.tic(func_name); }

  void toc(const std::string & func_name, const std::string & white_spaces)
  {
    const double elapsed_time = stop_watch_.toc(func_name);
    *this << white_spaces << func_name << ":= " << elapsed_time << " [ms]";
    accumulated_time = elapsed_time;
    endLine();
  }

  std::string getLog() const { return accumulated_msg; }

  std::string accumulated_msg = "\n";
  std::stringstream latest_stream;

  double getAccumulatedTime() const { return accumulated_time; }

  double accumulated_time{0.0};

  autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;
};

struct CommonParam
{
  CommonParam() = default;
  explicit CommonParam(rclcpp::Node * node)
  {
    output_backward_traj_length =
      node->declare_parameter<double>("common.output_backward_traj_length");
    output_delta_arc_length = node->declare_parameter<double>("common.output_delta_arc_length");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    using autoware_utils::update_param;

    // common
    update_param<double>(
      parameters, "common.output_backward_traj_length", output_backward_traj_length);
    update_param<double>(parameters, "common.output_delta_arc_length", output_delta_arc_length);
  }

  double output_delta_arc_length;
  double output_backward_traj_length;
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
    using autoware_utils::update_param;
    update_param<double>(parameters, "ego_nearest_dist_threshold", dist_threshold);
    update_param<double>(parameters, "ego_nearest_yaw_threshold", yaw_threshold);
  }

  double dist_threshold{0.0};
  double yaw_threshold{0.0};
};
}  // namespace autoware::path_smoother

#endif  // AUTOWARE__PATH_SMOOTHER__COMMON_STRUCTS_HPP_
