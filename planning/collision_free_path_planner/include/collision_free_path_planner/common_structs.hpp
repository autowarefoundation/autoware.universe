// Copyright 2021 Tier IV, Inc.
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
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <vector>

namespace collision_free_path_planner
{
struct PlannerData
{
  Path path;
  geometry_msgs::msg::Pose ego_pose{};
  double ego_vel{};
  std::vector<PredictedObject> objects{};
  bool enable_avoidance{false};
};

struct ReferencePoint;

struct Bounds;
using VehicleBounds = std::vector<Bounds>;
using SequentialBounds = std::vector<Bounds>;

using BoundsCandidates = std::vector<Bounds>;
using SequentialBoundsCandidates = std::vector<BoundsCandidates>;

struct CVMaps
{
  cv::Mat drivable_area;
  cv::Mat clearance_map;
  cv::Mat only_objects_clearance_map;
  cv::Mat area_with_objects_map;
  nav_msgs::msg::MapMetaData map_info;
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
  double clearance_for_fixing;
  double clearance_for_straight_line;
  double clearance_for_joint;
  double clearance_for_only_smoothing;

  int num_joint_buffer_points;
  int num_offset_for_begin_idx;

  double delta_arc_length_for_eb;
  int num_sampling_points_for_eb;
};

struct ConstrainRectangle
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
    StreamWithPrint & operator<<(const std::string & s)
    {
      sstream << s;
      if (s.back() == '\n') {
        std::string tmp_str = sstream.str();
        debug_str += tmp_str;

        if (enable_calculation_time_info) {
          tmp_str.pop_back();  // NOTE: remove '\n' which is unnecessary for RCLCPP_INFO_STREAM
          RCLCPP_INFO_STREAM(rclcpp::get_logger("collision_free_path_planner.time"), tmp_str);
        }
        sstream.str("");
      }
      return *this;
    }

    StreamWithPrint & operator<<(const double d)
    {
      sstream << d;
      return *this;
    }

    std::string getString() const { return debug_str; }

    bool enable_calculation_time_info;
    std::string debug_str = "\n";
    std::stringstream sstream;
  };

  void reset(const bool enable_calculation_time_info)
  {
    msg_stream = StreamWithPrint{};
    msg_stream.enable_calculation_time_info = enable_calculation_time_info;
  }

  // string stream for calculation time
  StreamWithPrint msg_stream;

  // settting
  size_t mpt_visualize_sampling_num;
  geometry_msgs::msg::Pose ego_pose;
  std::vector<double> vehicle_circle_radiuses;
  std::vector<double> vehicle_circle_longitudinal_offsets;

  // costmap
  std::vector<PredictedObject> avoiding_objects;
  // eb
  std::vector<ConstrainRectangle> constrain_rectangles;
  std::vector<TrajectoryPoint> eb_traj;
  // mpt
  std::vector<ReferencePoint> ref_points;
  std::vector<geometry_msgs::msg::Pose> mpt_ref_poses;
  SequentialBoundsCandidates sequential_bounds_candidates;
  std::vector<double> lateral_errors;
  std::vector<std::vector<geometry_msgs::msg::Pose>> vehicle_circles_pose;

  boost::optional<geometry_msgs::msg::Pose> stop_pose_by_drivable_area = boost::none;

  std::vector<TrajectoryPoint> extended_fixed_traj;
  std::vector<TrajectoryPoint> extended_non_fixed_traj;

  // std::vector<TrajectoryPoint> mpt_fixed_traj;
  // std::vector<TrajectoryPoint> mpt_ref_traj;
  // std::vector<TrajectoryPoint> mpt_traj;
};

struct TrajectoryParam
{
  TrajectoryParam() = default;
  TrajectoryParam(rclcpp::Node * node, const double vehicle_width)
  {  // trajectory parameter
    num_sampling_points = node->declare_parameter<int>("common.num_sampling_points");
    output_traj_length = node->declare_parameter<double>("common.output_traj_length");
    forward_fixing_min_distance =
      node->declare_parameter<double>("common.forward_fixing_min_distance");
    forward_fixing_min_time = node->declare_parameter<double>("common.forward_fixing_min_time");
    output_backward_traj_length =
      node->declare_parameter<double>("common.output_backward_traj_length");
    output_delta_arc_length = node->declare_parameter<double>("common.output_delta_arc_length");

    delta_dist_threshold_for_closest_point =
      node->declare_parameter<double>("common.delta_dist_threshold_for_closest_point");
    delta_yaw_threshold_for_closest_point =
      node->declare_parameter<double>("common.delta_yaw_threshold_for_closest_point");
    delta_yaw_threshold_for_straight =
      node->declare_parameter<double>("common.delta_yaw_threshold_for_straight");

    // TODO(murooka) tune this param when avoiding with collision_free_path_planner
    center_line_width = vehicle_width;
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    using tier4_autoware_utils::updateParam;

    // common
    updateParam<int>(parameters, "common.num_sampling_points", num_sampling_points);
    updateParam<double>(parameters, "common.output_traj_length", output_traj_length);
    updateParam<double>(
      parameters, "common.forward_fixing_min_distance", forward_fixing_min_distance);
    updateParam<double>(parameters, "common.forward_fixing_min_time", forward_fixing_min_time);
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

  // output
  double output_traj_length;
  double output_delta_arc_length;
  double output_backward_traj_length;

  int num_sampling_points;

  double delta_dist_threshold_for_closest_point;
  double delta_yaw_threshold_for_closest_point;
  double delta_yaw_threshold_for_straight;

  double forward_fixing_min_distance;
  double forward_fixing_min_time;
  double center_line_width;
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

    // common
    updateParam<double>(parameters, "ego_nearest_dist_threshold", dist_threshold);
    updateParam<double>(parameters, "ego_nearest_yaw_threshold", yaw_threshold);
  }

  double dist_threshold{0.0};
  double yaw_threshold{0.0};
};

}  // namespace collision_free_path_planner

#endif  // COLLISION_FREE_PATH_PLANNER__COMMON_STRUCTS_HPP_
