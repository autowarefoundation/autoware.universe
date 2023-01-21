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

struct EBParam
{
  // qp
  struct QPParam
  {
    int max_iteration;
    double eps_abs;
    double eps_rel;
  };
  QPParam qp_param;

  // clearance
  int num_joint_points;
  double clearance_for_fix;
  double clearance_for_joint;
  double clearance_for_smooth;

  double delta_arc_length;
  int num_points;

  EBParam() = default;
  explicit EBParam(rclcpp::Node * node)
  {
    {  // common
      delta_arc_length = node->declare_parameter<double>("advanced.eb.common.delta_arc_length");
      num_points = node->declare_parameter<int>("advanced.eb.common.num_points");
    }

    {  // clearance
      num_joint_points = node->declare_parameter<int>("advanced.eb.clearance.num_joint_points");
      clearance_for_fix =
        node->declare_parameter<double>("advanced.eb.clearance.clearance_for_fix");
      clearance_for_joint =
        node->declare_parameter<double>("advanced.eb.clearance.clearance_for_joint");
      clearance_for_smooth =
        node->declare_parameter<double>("advanced.eb.clearance.clearance_for_smooth");
    }

    {  // qp
      qp_param.max_iteration = node->declare_parameter<int>("advanced.eb.qp.max_iteration");
      qp_param.eps_abs = node->declare_parameter<double>("advanced.eb.qp.eps_abs");
      qp_param.eps_rel = node->declare_parameter<double>("advanced.eb.qp.eps_rel");
    }
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    using tier4_autoware_utils::updateParam;

    {  // common
      updateParam<double>(parameters, "advanced.eb.common.delta_arc_length", delta_arc_length);
      updateParam<int>(parameters, "advanced.eb.common.num_points", num_points);
    }

    {  // clearance
      updateParam<int>(parameters, "advanced.eb.clearance.num_joint_points", num_joint_points);
      updateParam<double>(parameters, "advanced.eb.clearance.clearance_for_fix", clearance_for_fix);
      updateParam<double>(
        parameters, "advanced.eb.clearance.clearance_for_joint", clearance_for_joint);
      updateParam<double>(
        parameters, "advanced.eb.clearance.clearance_for_smooth", clearance_for_smooth);
    }

    {  // qp
      updateParam<int>(parameters, "advanced.eb.qp.max_iteration", qp_param.max_iteration);
      updateParam<double>(parameters, "advanced.eb.qp.eps_abs", qp_param.eps_abs);
      updateParam<double>(parameters, "advanced.eb.qp.eps_rel", qp_param.eps_rel);
    }
  }
};

struct ConstraintRectangle
{
  geometry_msgs::msg::Point top_left;
  geometry_msgs::msg::Point top_right;
  geometry_msgs::msg::Point bottom_left;
  geometry_msgs::msg::Point bottom_right;
  double velocity;
  bool is_empty_driveable_area = false;
  bool is_including_only_smooth_range = true;
};

struct DebugData
{
  struct StreamWithPrint
  {
    template <typename T>
    StreamWithPrint & operator<<(const T & msg)
    {
      sstream << msg;
      return *this;
    }

    void endLine()
    {
      const auto msg = sstream.str();
      accumulated_msg += msg + "\n";

      if (enable_calculation_time_info) {
        // msg.pop_back();  // NOTE: remove '\n' which is unnecessary for RCLCPP_INFO_STREAM
        RCLCPP_INFO_STREAM(rclcpp::get_logger("collision_free_path_planner.time"), msg);
      }
      sstream.str("");
    }

    bool enable_calculation_time_info;
    std::string accumulated_msg = "\n";
    std::stringstream sstream;
  };

  void reset(const bool enable_calculation_time_info)
  {
    msg_stream = StreamWithPrint{};
    msg_stream.enable_calculation_time_info = enable_calculation_time_info;
    stop_pose_by_drivable_area = {};
    vehicle_circles_pose.clear();
  }

  void tic(const std::string & func_name) { stop_watch_.tic(func_name); }

  void toc(const std::string & func_name, const std::string & white_spaces)
  {
    const double elapsed_time = stop_watch_.toc(func_name);
    msg_stream << white_spaces << func_name << ":= " << elapsed_time << " [ms]";
    msg_stream.endLine();
  }

  std::string getAccumulatedTimeString() const { return msg_stream.accumulated_msg; }

  // string stream for calculation time
  StreamWithPrint msg_stream;

  // settting
  size_t mpt_visualize_sampling_num;
  geometry_msgs::msg::Pose ego_pose;
  std::vector<double> vehicle_circle_radiuses;
  std::vector<double> vehicle_circle_longitudinal_offsets;

  // eb
  std::vector<ConstraintRectangle> constraint_rectangles;
  std::vector<TrajectoryPoint> eb_traj;

  // mpt
  std::vector<ReferencePoint> ref_points;
  std::vector<std::vector<geometry_msgs::msg::Pose>> vehicle_circles_pose;

  std::optional<geometry_msgs::msg::Pose> stop_pose_by_drivable_area = {};

  std::vector<TrajectoryPoint> extended_traj_points;

  tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;
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
