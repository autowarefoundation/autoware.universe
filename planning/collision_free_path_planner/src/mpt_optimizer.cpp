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

#include "collision_free_path_planner/mpt_optimizer.hpp"

#include "collision_free_path_planner/utils/geometry_utils.hpp"
#include "collision_free_path_planner/utils/trajectory_utils.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/motion_utils.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "boost/assign/list_of.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <tuple>

namespace collision_free_path_planner
{
namespace
{
std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByUniformModel(
  const vehicle_info_util::VehicleInfo & vehicle_info, const size_t circle_num,
  const double radius_ratio)
{
  const double radius = std::hypot(
                          vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
                          vehicle_info.vehicle_width_m / 2.0) *
                        radius_ratio;
  std::vector<double> radiuses(circle_num, radius);

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  std::vector<double> longitudinal_offsets;
  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(
      unit_lon_length / 2.0 + unit_lon_length * i - vehicle_info.rear_overhang_m);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByBicycleModel(
  const vehicle_info_util::VehicleInfo & vehicle_info, const size_t circle_num,
  const double rear_radius_ratio, const double front_radius_ratio)
{
  // 1st circle (rear wheel)
  const double rear_radius = vehicle_info.vehicle_width_m / 2.0 * rear_radius_ratio;
  const double rear_lon_offset = 0.0;

  // 2nd circle (front wheel)
  const double front_radius =
    std::hypot(
      vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
      vehicle_info.vehicle_width_m / 2.0) *
    front_radius_ratio;

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  const double front_lon_offset =
    unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_info.rear_overhang_m;

  return {{rear_radius, front_radius}, {rear_lon_offset, front_lon_offset}};
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> extractBounds(
  const std::vector<ReferencePoint> & ref_points, const size_t l_idx, const double offset)
{
  Eigen::VectorXd ub_vec(ref_points.size());
  Eigen::VectorXd lb_vec(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ub_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).upper_bound + offset;
    lb_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).lower_bound - offset;
  }
  return {ub_vec, lb_vec};
}

std::vector<double> toStdVector(const Eigen::VectorXd & eigen_vec)
{
  return {eigen_vec.data(), eigen_vec.data() + eigen_vec.rows()};
}

[[maybe_unused]] std::vector<ReferencePoint> resampleRefPoints(
  const std::vector<ReferencePoint> & ref_points, const double interval)
{
  const auto traj_points = trajectory_utils::convertToTrajectoryPoints(ref_points);
  const auto resampled_traj_points =
    trajectory_utils::resampleTrajectoryPoints(traj_points, interval);
  return trajectory_utils::convertToReferencePoints(resampled_traj_points);
}

/*
std::optional<geometry_msgs::msg::Point> intersect(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  const double det = (p1.x - p2.x) * (p4.y - p3.y) - (p4.x - p3.x) * (p1.y - p2.y);
  if (det == 0.0) {
    return std::nullopt;
  }

  const double t = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  if (t < 0 || 1 < t) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point intersect_point;
  intersect_point.x = t * p1.x + (1.0 - t) * p2.x;
  intersect_point.y = t * p1.y + (1.0 - t) * p2.y;
  return intersect_point;
}
*/

std::optional<geometry_msgs::msg::Point> intersect(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Line = boost::geometry::model::linestring<Point>;

  const Line l1{{p1.x, p1.y}, {p2.x, p2.y}};
  const Line l2{{p3.x, p3.y}, {p4.x, p4.y}};

  std::vector<Point> intersect_points;
  boost::geometry::intersection(l1, l2, intersect_points);
  if (intersect_points.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point intersect_point;
  intersect_point.x = intersect_points.front().x();
  intersect_point.y = intersect_points.front().y();
  return intersect_point;
}

// NOTE: left is positive, and right is negative
double calcLateralDistToBounds(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & bound,
  const double additional_offset, const bool is_left_bound = true)
{
  const double max_lat_offset = 5.0;

  const double lat_offset = is_left_bound ? max_lat_offset : -max_lat_offset;
  const auto lat_offset_point =
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, lat_offset, 0.0).position;

  double dist_to_bound = max_lat_offset;
  for (size_t i = 0; i < bound.size() - 1; ++i) {
    const auto intersect_point =
      intersect(pose.position, lat_offset_point, bound.at(i), bound.at(i + 1));
    if (intersect_point) {
      const double tmp_dist =
        tier4_autoware_utils::calcDistance2d(pose.position, *intersect_point) - additional_offset;
      dist_to_bound = std::min(tmp_dist, dist_to_bound);
    }
  }

  return is_left_bound ? dist_to_bound : -dist_to_bound;
}
}  // namespace

MPTOptimizer::MPTOptimizer(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const vehicle_info_util::VehicleInfo & vehicle_info, const TrajectoryParam & traj_param,
  const std::shared_ptr<DebugData> debug_data_ptr)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  traj_param_(traj_param),
  vehicle_info_(vehicle_info),
  debug_data_ptr_(debug_data_ptr)
{
  // mpt param
  initializeMPTParam(node, vehicle_info);

  // state equation generator
  state_equation_generator_ =
    StateEquationGenerator(vehicle_info_.wheel_base_m, mpt_param_.max_steer_rad);

  // osqp solver
  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(osqp_epsilon_);

  // publisher
  debug_fixed_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_ref_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_traj", 1);
}

void MPTOptimizer::initializeMPTParam(
  rclcpp::Node * node, const vehicle_info_util::VehicleInfo & vehicle_info)
{
  mpt_param_ = MPTParam{};

  {  // option
    mpt_param_.steer_limit_constraint =
      node->declare_parameter<bool>("mpt.option.steer_limit_constraint");
    mpt_param_.fix_points_around_ego =
      node->declare_parameter<bool>("mpt.option.fix_points_around_ego");
    mpt_param_.enable_warm_start = node->declare_parameter<bool>("mpt.option.enable_warm_start");
    mpt_param_.enable_manual_warm_start =
      node->declare_parameter<bool>("mpt.option.enable_manual_warm_start");
    mpt_visualize_sampling_num_ = node->declare_parameter<int>("mpt.option.visualize_sampling_num");
    mpt_param_.is_fixed_point_single =
      node->declare_parameter<bool>("mpt.option.is_fixed_point_single");
  }

  {  // common
    mpt_param_.num_points = node->declare_parameter<int>("mpt.common.num_points");
    mpt_param_.delta_arc_length = node->declare_parameter<double>("mpt.common.delta_arc_length");
  }

  // kinematics
  mpt_param_.max_steer_rad = vehicle_info.max_steer_angle_rad;

  // By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
  // The 0.8 scale is adopted as it performed the best.
  constexpr double default_wheel_base_ratio = 0.8;
  mpt_param_.optimization_center_offset = node->declare_parameter<double>(
    "mpt.kinematics.optimization_center_offset",
    vehicle_info_.wheel_base_m * default_wheel_base_ratio);

  // bounds search
  mpt_param_.bounds_search_widths =
    node->declare_parameter<std::vector<double>>("advanced.mpt.bounds_search_widths");

  // collision free constraints
  mpt_param_.l_inf_norm =
    node->declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.l_inf_norm");
  mpt_param_.soft_constraint =
    node->declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.soft_constraint");
  mpt_param_.hard_constraint =
    node->declare_parameter<bool>("advanced.mpt.collision_free_constraints.option.hard_constraint");

  {  // vehicle_circles
     // NOTE: Vehicle shape for collision free constraints is considered as a set of circles
    vehicle_circle_method_ = node->declare_parameter<std::string>(
      "advanced.mpt.collision_free_constraints.vehicle_circles.method");

    if (vehicle_circle_method_ == "uniform_circle") {
      vehicle_circle_num_for_calculation_ = node->declare_parameter<int>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.num");
      vehicle_circle_radius_ratios_.push_back(node->declare_parameter<double>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio"));

      std::tie(mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
        calcVehicleCirclesByUniformModel(
          vehicle_info_, vehicle_circle_num_for_calculation_,
          vehicle_circle_radius_ratios_.front());
    } else if (vehicle_circle_method_ == "bicycle_model") {
      vehicle_circle_num_for_calculation_ = node->declare_parameter<int>(
        "advanced.mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_"
        "calculation");

      vehicle_circle_radius_ratios_.push_back(
        node->declare_parameter<double>("advanced.mpt.collision_free_constraints.vehicle_circles."
                                        "bicycle_model.rear_radius_ratio"));
      vehicle_circle_radius_ratios_.push_back(
        node->declare_parameter<double>("advanced.mpt.collision_free_constraints.vehicle_circles."
                                        "bicycle_model.front_radius_ratio"));

      std::tie(mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
        calcVehicleCirclesByBicycleModel(
          vehicle_info_, vehicle_circle_num_for_calculation_, vehicle_circle_radius_ratios_.front(),
          vehicle_circle_radius_ratios_.back());
    } else {
      throw std::invalid_argument(
        "advanced.mpt.collision_free_constraints.vehicle_circles.num parameter is invalid.");
    }
  }

  {  // clearance
    mpt_param_.hard_clearance_from_road =
      node->declare_parameter<double>("advanced.mpt.clearance.hard_clearance_from_road");
    mpt_param_.soft_clearance_from_road =
      node->declare_parameter<double>("advanced.mpt.clearance.soft_clearance_from_road");
  }

  {  // weight
    mpt_param_.soft_avoidance_weight =
      node->declare_parameter<double>("advanced.mpt.weight.soft_avoidance_weight");

    mpt_param_.lat_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.lat_error_weight");
    mpt_param_.yaw_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.yaw_error_weight");
    mpt_param_.yaw_error_rate_weight =
      node->declare_parameter<double>("advanced.mpt.weight.yaw_error_rate_weight");
    mpt_param_.steer_input_weight =
      node->declare_parameter<double>("advanced.mpt.weight.steer_input_weight");
    mpt_param_.steer_rate_weight =
      node->declare_parameter<double>("advanced.mpt.weight.steer_rate_weight");

    mpt_param_.obstacle_avoid_lat_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_lat_error_weight");
    mpt_param_.obstacle_avoid_yaw_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_yaw_error_weight");
    mpt_param_.obstacle_avoid_steer_input_weight =
      node->declare_parameter<double>("advanced.mpt.weight.obstacle_avoid_steer_input_weight");
    mpt_param_.near_objects_length =
      node->declare_parameter<double>("advanced.mpt.weight.near_objects_length");

    mpt_param_.terminal_lat_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.terminal_lat_error_weight");
    mpt_param_.terminal_yaw_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.terminal_yaw_error_weight");
    mpt_param_.terminal_path_lat_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.terminal_path_lat_error_weight");
    mpt_param_.terminal_path_yaw_error_weight =
      node->declare_parameter<double>("advanced.mpt.weight.terminal_path_yaw_error_weight");
  }

  // update debug data
  debug_data_ptr_->vehicle_circle_radiuses = mpt_param_.vehicle_circle_radiuses;
  debug_data_ptr_->vehicle_circle_longitudinal_offsets =
    mpt_param_.vehicle_circle_longitudinal_offsets;
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_visualize_sampling_num_;
}

void MPTOptimizer::reset(const bool enable_debug_info, const TrajectoryParam & traj_param)
{
  enable_debug_info_ = enable_debug_info;
  traj_param_ = traj_param;
}

void MPTOptimizer::resetPrevData() { prev_ref_points_ptr_ = nullptr; }

void MPTOptimizer::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option
    updateParam<bool>(
      parameters, "mpt.option.steer_limit_constraint", mpt_param_.steer_limit_constraint);
    updateParam<bool>(
      parameters, "mpt.option.fix_points_around_ego", mpt_param_.fix_points_around_ego);
    updateParam<bool>(parameters, "mpt.option.enable_warm_start", mpt_param_.enable_warm_start);
    updateParam<bool>(
      parameters, "mpt.option.enable_manual_warm_start", mpt_param_.enable_manual_warm_start);
    updateParam<int>(parameters, "mpt.option.visualize_sampling_num", mpt_visualize_sampling_num_);
    updateParam<bool>(
      parameters, "mpt.option.option.is_fixed_point_single", mpt_param_.is_fixed_point_single);
  }

  // common
  updateParam<int>(parameters, "mpt.common.num_points", mpt_param_.num_points);
  updateParam<double>(parameters, "mpt.common.delta_arc_length", mpt_param_.delta_arc_length);

  // kinematics
  updateParam<double>(
    parameters, "mpt.kinematics.optimization_center_offset", mpt_param_.optimization_center_offset);

  // collision_free_constraints
  updateParam<bool>(
    parameters, "advanced.mpt.collision_free_constraints.option.l_inf_norm", mpt_param_.l_inf_norm);
  updateParam<bool>(
    parameters, "advanced.mpt.collision_free_constraints.option.soft_constraint",
    mpt_param_.soft_constraint);
  updateParam<bool>(
    parameters, "advanced.mpt.collision_free_constraints.option.hard_constraint",
    mpt_param_.hard_constraint);

  {  // vehicle_circles
    // NOTE: Changing method is not supported
    // updateParam<std::string>(
    //   parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.method",
    //   vehicle_circle_method_);

    // TODO(murooka) add bicycle
    if (vehicle_circle_method_ == "uniform_circle") {
      updateParam<int>(
        parameters, "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.num",
        vehicle_circle_num_for_calculation_);
      updateParam<double>(
        parameters,
        "advanced.mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio",
        vehicle_circle_radius_ratios_.front());

      std::tie(mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
        calcVehicleCirclesByUniformModel(
          vehicle_info_, vehicle_circle_num_for_calculation_,
          vehicle_circle_radius_ratios_.front());
    } else if (vehicle_circle_method_ == "bicycle_model") {
      updateParam<int>(
        parameters,
        "advanced.mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_calculation",
        vehicle_circle_num_for_calculation_);
      updateParam<double>(
        parameters,
        "advanced.mpt.collision_free_constraints.vehicle_circles.bicycle_model.rear_radius_ratio",
        vehicle_circle_radius_ratios_.front());
      updateParam<double>(
        parameters,
        "advanced.mpt.collision_free_constraints.vehicle_circles."
        "bicycle_model.front_radius_ratio",
        vehicle_circle_radius_ratios_.back());

      std::tie(mpt_param_.vehicle_circle_radiuses, mpt_param_.vehicle_circle_longitudinal_offsets) =
        calcVehicleCirclesByBicycleModel(
          vehicle_info_, vehicle_circle_num_for_calculation_, vehicle_circle_radius_ratios_.front(),
          vehicle_circle_radius_ratios_.back());
    } else {
      throw std::invalid_argument("vehicle_circle_method_ is invalid.");
    }
  }

  {  // clearance
    updateParam<double>(
      parameters, "advanced.mpt.clearance.hard_clearance_from_road",
      mpt_param_.hard_clearance_from_road);
    updateParam<double>(
      parameters, "advanced.mpt.clearance.soft_clearance_from_road",
      mpt_param_.soft_clearance_from_road);
  }

  {  // weight
    updateParam<double>(
      parameters, "advanced.mpt.weight.soft_avoidance_weight", mpt_param_.soft_avoidance_weight);

    updateParam<double>(
      parameters, "advanced.mpt.weight.lat_error_weight", mpt_param_.lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.yaw_error_weight", mpt_param_.yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.yaw_error_rate_weight", mpt_param_.yaw_error_rate_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.steer_input_weight", mpt_param_.steer_input_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.steer_rate_weight", mpt_param_.steer_rate_weight);

    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_lat_error_weight",
      mpt_param_.obstacle_avoid_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_yaw_error_weight",
      mpt_param_.obstacle_avoid_yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.obstacle_avoid_steer_input_weight",
      mpt_param_.obstacle_avoid_steer_input_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.near_objects_length", mpt_param_.near_objects_length);

    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_lat_error_weight",
      mpt_param_.terminal_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_yaw_error_weight",
      mpt_param_.terminal_yaw_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_path_lat_error_weight",
      mpt_param_.terminal_path_lat_error_weight);
    updateParam<double>(
      parameters, "advanced.mpt.weight.terminal_path_yaw_error_weight",
      mpt_param_.terminal_path_yaw_error_weight);
  }

  // update debug data
  debug_data_ptr_->vehicle_circle_radiuses = mpt_param_.vehicle_circle_radiuses;
  debug_data_ptr_->vehicle_circle_longitudinal_offsets =
    mpt_param_.vehicle_circle_longitudinal_offsets;
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_visualize_sampling_num_;
}

std::optional<MPTTrajs> MPTOptimizer::getModelPredictiveTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points)
{
  debug_data_ptr_->tic(__func__);

  const auto & p = planner_data;
  const auto & traj_points = p.traj_points;

  // 1. calculate reference points
  auto ref_points = calcReferencePoints(planner_data, smoothed_points);
  if (ref_points.empty()) {
    logInfo("return std::nullopt since ref_points is empty");
    return std::nullopt;
  } else if (ref_points.size() == 1) {
    logInfo("return std::nullopt since ref_points.size() == 1");
    return std::nullopt;
  }

  // 2. calculate B and W matrices where x = B u + W
  const auto mpt_mat = state_equation_generator_.calcMatrix(ref_points, *debug_data_ptr_);

  // 3. calculate Q and R matrices where J(x, u) = x^t Q x + u^t R u
  const auto val_mat = calcValueMatrix(ref_points, traj_points);

  // 4. get objective matrix
  const auto obj_mat = getObjectiveMatrix(mpt_mat, val_mat, ref_points);

  // 5. get constraints matrix
  const auto const_mat = getConstraintMatrix(mpt_mat, ref_points);

  // 6. optimize steer angles
  const auto optimized_steer_angles = calcOptimizedSteerAngles(ref_points, obj_mat, const_mat);
  if (!optimized_steer_angles) {
    logInfo("return std::nullopt since could not solve qp");
    return std::nullopt;
  }

  for (const auto & a : *optimized_steer_angles) {
    std::cerr << a << " ";
  }
  std::cerr << std::endl;

  // 5. convert to points
  const auto mpt_points = calcMPTPoints(ref_points, *optimized_steer_angles, mpt_mat);

  // 6. publish trajectories for debug
  publishDebugTrajectories(p.header, ref_points, mpt_points);

  debug_data_ptr_->toc(__func__, "      ");

  const auto mpt_trajs = MPTTrajs{ref_points, mpt_points};
  prev_ref_points_ptr_ = std::make_shared<std::vector<ReferencePoint>>(ref_points);

  return mpt_trajs;
}

std::vector<ReferencePoint> MPTOptimizer::calcReferencePoints(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points) const
{
  debug_data_ptr_->tic(__func__);

  const auto & p = planner_data;

  const double forward_traj_length = mpt_param_.num_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  // 1. resample and convert smoothed points type from trajectory points to reference points
  auto ref_points = [&]() {
    const auto resampled_smoothed_points =
      trajectory_utils::resampleTrajectoryPoints(smoothed_points, mpt_param_.delta_arc_length);
    return trajectory_utils::convertToReferencePoints(resampled_smoothed_points);
  }();

  // 2. crop forward and backward with margin, and calculate spline interpolation
  const double tmp_margin = 10.0;
  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);
  ref_points = trajectory_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    -backward_traj_length - tmp_margin);
  SplineInterpolationPoints2d ref_points_spline(ref_points);

  // 3. calculate orientation and curvature
  // NOTE: Calculating orientation and curvature requirs points
  // around the target points to be smooth.
  //       Therefore, crop margin is required.
  updateOrientation(ref_points, ref_points_spline);
  updateCurvature(ref_points, ref_points_spline);

  // crop backward, and calculate spline interpolation
  // NOTE: After cropped, spline interpolation must be updated.
  ref_points = trajectory_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    -backward_traj_length);
  ref_points_spline = SplineInterpolationPoints2d(ref_points);

  // must be after backward cropping
  // NOTE: New front point may be added. Resample is required.
  updateFixedPoint(ref_points);
  ref_points_spline = SplineInterpolationPoints2d(ref_points);

  // set bounds information
  updateBounds(ref_points, p.left_bound, p.right_bound);
  updateVehicleBounds(ref_points, ref_points_spline);

  updateArcLength(ref_points);

  // set extra information (alpha and has_object_collision)
  // NOTE: This must be after bounds calculation and updateOrientation
  // must be after updateArcLength
  calcExtraPoints(ref_points);

  ref_points = trajectory_utils::clipForwardPoints(ref_points, 0, forward_traj_length);

  // bounds information is assigned to debug data after truncating reference points
  debug_data_ptr_->ref_points = ref_points;

  debug_data_ptr_->toc(__func__, "        ");

  return ref_points;
}

void MPTOptimizer::updateOrientation(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto yaw_vec = ref_points_spline.getSplineInterpolatedYaws();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw_vec.at(i));
  }
}

void MPTOptimizer::updateCurvature(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto curvature_vec = ref_points_spline.getSplineInterpolatedCurvatures();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).k = curvature_vec.at(i);
  }
}

void MPTOptimizer::updateFixedPoint(std::vector<ReferencePoint> & ref_points) const
{
  if (!prev_ref_points_ptr_) {
    // no fixed point
    return;
  }

  const size_t prev_ref_front_seg_idx = trajectory_utils::findEgoSegmentIndex(
    *prev_ref_points_ptr_, tier4_autoware_utils::getPose(ref_points.front()), ego_nearest_param_);
  const size_t prev_ref_front_point_idx = prev_ref_front_seg_idx;

  const auto & prev_ref_front_point = prev_ref_points_ptr_->at(prev_ref_front_point_idx);

  // TODO(murooka) check deviation

  // update front pose of ref_points
  trajectory_utils::updateFrontPointForFix(
    ref_points, prev_ref_front_point.pose, mpt_param_.delta_arc_length);
  ref_points.front().fix_kinematic_state = prev_ref_front_point.optimized_kinematic_state;
}

// cost function: J = x' Q x + u' R u
MPTOptimizer::ValueMatrix MPTOptimizer::calcValueMatrix(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  debug_data_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();

  const size_t D_v = D_x + (N_ref - 1) * D_u;

  const bool is_containing_path_terminal_point = trajectory_utils::isNearLastPathPoint(
    ref_points.back(), traj_points, 0.0001, traj_param_.delta_yaw_threshold_for_closest_point);

  // update Q
  Eigen::SparseMatrix<double> Q_sparse_mat(D_x * N_ref, D_x * N_ref);
  std::vector<Eigen::Triplet<double>> Q_triplet_vec;
  for (size_t i = 0; i < N_ref; ++i) {
    const auto adaptive_error_weight = [&]() -> std::array<double, 2> {
      if (ref_points.at(i).near_objects) {
        return {
          mpt_param_.obstacle_avoid_lat_error_weight, mpt_param_.obstacle_avoid_yaw_error_weight};
      } else if (i == N_ref - 1 && is_containing_path_terminal_point) {
        return {
          mpt_param_.terminal_path_lat_error_weight, mpt_param_.terminal_path_yaw_error_weight};
      } else if (i == N_ref - 1) {
        return {mpt_param_.terminal_lat_error_weight, mpt_param_.terminal_yaw_error_weight};
      }
      // NOTE: may be better to add decreasing weights in a narrow and sharp curve
      // else if (std::abs(ref_points[i].k) > 0.3) {
      //   return {0.0, 0.0};
      // }

      return {mpt_param_.lat_error_weight, mpt_param_.yaw_error_weight};
    }();

    const double adaptive_lat_error_weight = adaptive_error_weight.at(0);
    const double adaptive_yaw_error_weight = adaptive_error_weight.at(1);

    Q_triplet_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, adaptive_lat_error_weight));
    Q_triplet_vec.push_back(
      Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, adaptive_yaw_error_weight));
  }
  Q_sparse_mat.setFromTriplets(Q_triplet_vec.begin(), Q_triplet_vec.end());

  // update R
  Eigen::SparseMatrix<double> R_sparse_mat(D_v, D_v);
  std::vector<Eigen::Triplet<double>> R_triplet_vec;
  for (size_t i = 0; i < N_ref - 1; ++i) {
    const double adaptive_steer_weight = ref_points.at(i).near_objects
                                           ? mpt_param_.obstacle_avoid_steer_input_weight
                                           : mpt_param_.steer_input_weight;
    R_triplet_vec.push_back(
      Eigen::Triplet<double>(D_x + D_u * i, D_x + D_u * i, adaptive_steer_weight));
  }
  addSteerWeightR(R_triplet_vec, ref_points);

  R_sparse_mat.setFromTriplets(R_triplet_vec.begin(), R_triplet_vec.end());

  ValueMatrix m;
  m.Q = Q_sparse_mat;
  m.R = R_sparse_mat;

  debug_data_ptr_->toc(__func__, "        ");
  return m;
}

std::optional<Eigen::VectorXd> MPTOptimizer::calcOptimizedSteerAngles(
  const std::vector<ReferencePoint> & ref_points, const ObjectiveMatrix & obj_mat,
  const ConstraintMatrix & const_mat)
{
  debug_data_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_v = D_x + N_u;

  // for manual warm start, calculate initial solution
  const auto u0 = [&]() -> std::optional<Eigen::VectorXd> {
    if (mpt_param_.enable_manual_warm_start) {
      if (prev_ref_points_ptr_ && 1 < prev_ref_points_ptr_->size()) {
        return calcInitialSolutionForManualWarmStart(ref_points, *prev_ref_points_ptr_);
      }
    }
    return std::nullopt;
  }();

  // for manual start, update objective and constraint matrix
  const auto [updated_obj_mat, updated_const_mat] =
    updateMatrixForManualWarmStart(obj_mat, const_mat, u0);

  // calculate matrices for qp
  const Eigen::MatrixXd & H = updated_obj_mat.hessian;
  const Eigen::MatrixXd & A = updated_const_mat.linear;
  const auto f = toStdVector(updated_obj_mat.gradient);
  const auto upper_bound = toStdVector(updated_const_mat.upper_bound);
  const auto lower_bound = toStdVector(updated_const_mat.lower_bound);

  // initialize or update solver according to warm start
  debug_data_ptr_->tic("initOsqp");

  const autoware::common::osqp::CSC_Matrix P_csc =
    autoware::common::osqp::calCSCMatrixTrapezoidal(H);
  const autoware::common::osqp::CSC_Matrix A_csc = autoware::common::osqp::calCSCMatrix(A);
  if (mpt_param_.enable_warm_start && prev_mat_n_ == H.rows() && prev_mat_m_ == A.rows()) {
    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("mpt_optimizer"), enable_debug_info_, "warm start");
    osqp_solver_ptr_->updateCscP(P_csc);
    osqp_solver_ptr_->updateQ(f);
    osqp_solver_ptr_->updateCscA(A_csc);
    osqp_solver_ptr_->updateL(lower_bound);
    osqp_solver_ptr_->updateU(upper_bound);
  } else {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("mpt_optimizer"), enable_debug_info_, "no warm start");
    osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
      P_csc, A_csc, f, lower_bound, upper_bound, osqp_epsilon_);
  }
  prev_mat_n_ = H.rows();
  prev_mat_m_ = A.rows();

  debug_data_ptr_->toc("initOsqp", "          ");

  // solve qp
  debug_data_ptr_->tic("solveOsqp");
  const auto result = osqp_solver_ptr_->optimize();
  debug_data_ptr_->toc("solveOsqp", "          ");

  // check solution status
  const int solution_status = std::get<3>(result);
  if (solution_status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[MPT]");
    return std::nullopt;
  }

  // print iteration
  const int iteration_status = std::get<4>(result);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("mpt_optimizer"), enable_debug_info_, "iteration: %d", iteration_status);

  // get optimization result
  auto optimization_result =
    std::get<0>(result);  // NOTE: const cannot be added due to the next operation.
  const Eigen::VectorXd optimized_steer_angles =
    Eigen::Map<Eigen::VectorXd>(&optimization_result[0], N_v);

  debug_data_ptr_->toc(__func__, "        ");

  if (u0) {  // manual warm start
    return static_cast<Eigen::VectorXd>(optimized_steer_angles + u0->segment(0, N_v));
  }
  return optimized_steer_angles;
}

Eigen::VectorXd MPTOptimizer::calcInitialSolutionForManualWarmStart(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<ReferencePoint> & prev_ref_points) const
{
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_v = D_x + N_u;

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(N_v);

  const size_t seg_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    prev_ref_points, ref_points.front().pose, ego_nearest_param_.dist_threshold,
    ego_nearest_param_.yaw_threshold);

  // set previous lateral and yaw deviaton
  u0(0) = prev_ref_points.at(seg_idx).optimized_kinematic_state.lat;
  u0(1) = prev_ref_points.at(seg_idx).optimized_kinematic_state.yaw;

  // set previous steer angles
  for (size_t i = 0; i < N_u; ++i) {
    const size_t prev_target_idx = std::min(seg_idx + i, prev_ref_points.size() - 1);
    u0(D_x + i) = prev_ref_points.at(prev_target_idx).optimized_input;
  }

  return u0;
}

std::pair<MPTOptimizer::ObjectiveMatrix, MPTOptimizer::ConstraintMatrix>
MPTOptimizer::updateMatrixForManualWarmStart(
  const ObjectiveMatrix & obj_mat, const ConstraintMatrix & const_mat,
  const std::optional<Eigen::VectorXd> & u0) const
{
  if (!u0) {
    // not manual warm start
    return {obj_mat, const_mat};
  }

  const Eigen::MatrixXd & H = obj_mat.hessian;
  const Eigen::MatrixXd & A = const_mat.linear;

  auto updated_obj_mat = obj_mat;
  auto updated_const_mat = const_mat;

  Eigen::VectorXd & f = updated_obj_mat.gradient;
  Eigen::VectorXd & ub = updated_const_mat.upper_bound;
  Eigen::VectorXd & lb = updated_const_mat.lower_bound;

  // update gradient
  f += H * *u0;

  // update upper_bound and lower_bound
  const Eigen::VectorXd A_times_u0 = A * *u0;
  ub -= A_times_u0;
  lb -= A_times_u0;

  return {updated_obj_mat, updated_const_mat};
}

MPTOptimizer::ObjectiveMatrix MPTOptimizer::getObjectiveMatrix(
  const StateEquationGenerator::Matrix & mpt_mat, const ValueMatrix & val_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  debug_data_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();

  const size_t D_xn = D_x * N_ref;
  const size_t D_v = D_x + (N_ref - 1) * D_u;

  // generate T matrix and vector to shift optimization center
  //   define Z as time-series vector of shifted deviation error
  //   Z = sparse_T_mat * (B * U + W) + T_vec
  Eigen::SparseMatrix<double> sparse_T_mat(D_xn, D_xn);
  Eigen::VectorXd T_vec = Eigen::VectorXd::Zero(D_xn);
  std::vector<Eigen::Triplet<double>> triplet_T_vec;
  const double offset = mpt_param_.optimization_center_offset;

  for (size_t i = 0; i < N_ref; ++i) {
    const double alpha = ref_points.at(i).alpha;

    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x + 1, offset * std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, 1.0));

    T_vec(i * D_x) = -offset * std::sin(alpha);
  }
  sparse_T_mat.setFromTriplets(triplet_T_vec.begin(), triplet_T_vec.end());

  const Eigen::MatrixXd B = sparse_T_mat * mpt_mat.B;
  const Eigen::MatrixXd QB = val_mat.Q * B;
  const Eigen::MatrixXd R = val_mat.R;

  // min J(v) = min (v'Hv + v'f)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(D_v, D_v);
  H.triangularView<Eigen::Upper>() = B.transpose() * QB + R;
  H.triangularView<Eigen::Lower>() = H.transpose();

  // Eigen::VectorXd f = ((sparse_T_mat * mpt_mat.W + T_vec).transpose() * QB).transpose();
  Eigen::VectorXd f = (sparse_T_mat * mpt_mat.W + T_vec).transpose() * QB;

  const size_t N_avoid = mpt_param_.vehicle_circle_longitudinal_offsets.size();
  const size_t N_first_slack = [&]() -> size_t {
    if (mpt_param_.soft_constraint) {
      if (mpt_param_.l_inf_norm) {
        return 1;
      }
      return N_avoid;
    }
    return 0;
  }();

  // number of slack variables for one step
  const size_t N_slack = N_first_slack;

  // extend H for slack variables
  Eigen::MatrixXd full_H = Eigen::MatrixXd::Zero(D_v + N_ref * N_slack, D_v + N_ref * N_slack);
  full_H.block(0, 0, D_v, D_v) = H;

  // extend f for slack variables
  Eigen::VectorXd full_f(D_v + N_ref * N_slack);

  full_f.segment(0, D_v) = f;
  if (N_first_slack > 0) {
    full_f.segment(D_v, N_ref * N_first_slack) =
      mpt_param_.soft_avoidance_weight * Eigen::VectorXd::Ones(N_ref * N_first_slack);
  }

  ObjectiveMatrix obj_matrix;
  obj_matrix.hessian = full_H;
  obj_matrix.gradient = full_f;

  debug_data_ptr_->toc(__func__, "          ");
  return obj_matrix;
}

// Set constraint: lb <= Ax <= ub
// decision variable
// x := [u0, ..., uN-1 | z00, ..., z0N-1 | z10, ..., z1N-1 | z20, ..., z2N-1]
//   \in \mathbb{R}^{N * (N_vehicle_circle + 1)}
MPTOptimizer::ConstraintMatrix MPTOptimizer::getConstraintMatrix(
  const StateEquationGenerator::Matrix & mpt_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  debug_data_ptr_->tic(__func__);

  // NOTE: currently, add additional length to soft bounds approximately
  //       for hard bounds
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();

  const size_t N_u = (N_ref - 1) * D_u;
  const size_t D_v = D_x + N_u;

  const size_t N_avoid = mpt_param_.vehicle_circle_longitudinal_offsets.size();

  // number of slack variables for one step
  const size_t N_first_slack = [&]() -> size_t {
    if (mpt_param_.soft_constraint) {
      if (mpt_param_.l_inf_norm) {
        return 1;
      }
      return N_avoid;
    }
    return 0;
  }();

  // number of all slack variables is N_ref * N_slack
  const size_t N_slack = N_first_slack;

  const size_t A_cols = [&] {
    if (mpt_param_.soft_constraint) {
      return D_v + N_ref * N_slack;  // initial_state + steer + soft
    }
    return D_v;  // initial state + steer
  }();

  // calculate indices of fixed points
  std::vector<size_t> fixed_points_indices;
  for (size_t i = 0; i < N_ref; ++i) {
    if (ref_points.at(i).fix_kinematic_state) {
      fixed_points_indices.push_back(i);
    }
  }

  // calculate rows of A
  size_t A_rows = 0;
  if (mpt_param_.soft_constraint) {
    // 3 means slack variable constraints to be between lower and upper bounds, and positive.
    A_rows += 3 * N_ref * N_avoid;
  }
  if (mpt_param_.hard_constraint) {
    A_rows += N_ref * N_avoid;
  }
  A_rows += fixed_points_indices.size() * D_x;
  if (mpt_param_.steer_limit_constraint) {
    A_rows += N_u;
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_rows, A_cols);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(A_rows, -autoware::common::osqp::INF);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(A_rows, autoware::common::osqp::INF);
  size_t A_rows_end = 0;

  // CX = C(Bv + w) + C \in R^{N_ref, N_ref * D_x}
  for (size_t l_idx = 0; l_idx < N_avoid; ++l_idx) {
    // create C := [1 | l | O]
    Eigen::SparseMatrix<double> C_sparse_mat(N_ref, N_ref * D_x);
    std::vector<Eigen::Triplet<double>> C_triplet_vec;
    Eigen::VectorXd C_vec = Eigen::VectorXd::Zero(N_ref);

    // calculate C mat and vec
    for (size_t i = 0; i < N_ref; ++i) {
      const double beta = *ref_points.at(i).beta.at(l_idx);
      const double avoid_offset = mpt_param_.vehicle_circle_longitudinal_offsets.at(l_idx);

      C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x, 1.0 * std::cos(beta)));
      C_triplet_vec.push_back(
        Eigen::Triplet<double>(i, i * D_x + 1, avoid_offset * std::cos(beta)));
      C_vec(i) = avoid_offset * std::sin(beta);
    }
    C_sparse_mat.setFromTriplets(C_triplet_vec.begin(), C_triplet_vec.end());

    // calculate CB, and CW
    const Eigen::MatrixXd CB = C_sparse_mat * mpt_mat.B;
    const Eigen::VectorXd CW = C_sparse_mat * mpt_mat.W + C_vec;

    // calculate bounds
    const double bounds_offset =
      vehicle_info_.vehicle_width_m / 2.0 - mpt_param_.vehicle_circle_radiuses.at(l_idx);
    const auto & [part_ub, part_lb] = extractBounds(ref_points, l_idx, bounds_offset);

    // soft constraints
    if (mpt_param_.soft_constraint) {
      size_t A_offset_cols = D_v;
      const size_t A_blk_rows = 3 * N_ref;

      // A := [C * B | O | ... | O | I | O | ...
      //      -C * B | O | ... | O | I | O | ...
      //          O    | O | ... | O | I | O | ... ]
      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, A_cols);
      A_blk.block(0, 0, N_ref, D_v) = CB;
      A_blk.block(N_ref, 0, N_ref, D_v) = -CB;

      size_t local_A_offset_cols = A_offset_cols;
      if (!mpt_param_.l_inf_norm) {
        local_A_offset_cols += N_ref * l_idx;
      }
      A_blk.block(0, local_A_offset_cols, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(2 * N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);

      // lb := [lower_bound - CW
      //        CW - upper_bound
      //               O        ]
      Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(A_blk_rows);
      lb_blk.segment(0, N_ref) = -CW + part_lb;
      lb_blk.segment(N_ref, N_ref) = CW - part_ub;

      A_offset_cols += N_ref * N_first_slack;

      A.block(A_rows_end, 0, A_blk_rows, A_cols) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = lb_blk;

      A_rows_end += A_blk_rows;
    }

    // hard constraints
    if (mpt_param_.hard_constraint) {
      const size_t A_blk_rows = N_ref;

      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, A_cols);
      A_blk.block(0, 0, N_ref, N_ref) = CB;

      A.block(A_rows_end, 0, A_blk_rows, A_cols) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = part_lb - CW;
      ub.segment(A_rows_end, A_blk_rows) = part_ub - CW;

      A_rows_end += A_blk_rows;
    }
  }

  // fixed points constraint
  // CX = C(B v + w) where C extracts fixed points
  if (fixed_points_indices.size() > 0) {
    for (const size_t i : fixed_points_indices) {
      A.block(A_rows_end, 0, D_x, D_v) = mpt_mat.B.block(i * D_x, 0, D_x, D_v);

      lb.segment(A_rows_end, D_x) =
        ref_points[i].fix_kinematic_state->toEigenVector() - mpt_mat.W.segment(i * D_x, D_x);
      ub.segment(A_rows_end, D_x) =
        ref_points[i].fix_kinematic_state->toEigenVector() - mpt_mat.W.segment(i * D_x, D_x);

      A_rows_end += D_x;
    }
  }

  // steer max limit
  if (mpt_param_.steer_limit_constraint) {
    A.block(A_rows_end, D_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u, N_u);
    lb.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, -mpt_param_.max_steer_rad);
    ub.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, mpt_param_.max_steer_rad);

    A_rows_end += N_u;
  }

  ConstraintMatrix constraint_matrix;
  constraint_matrix.linear = A;
  constraint_matrix.lower_bound = lb;
  constraint_matrix.upper_bound = ub;

  debug_data_ptr_->toc(__func__, "          ");
  return constraint_matrix;
}

std::vector<TrajectoryPoint> MPTOptimizer::calcMPTPoints(
  std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & Uex,
  const StateEquationGenerator::Matrix & mpt_mat)
{
  debug_data_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref =
    ref_points
      .size();  // static_cast<size_t>(Uex.rows() - D_x) + 1; // TODO(murooka) remove this comment

  // predict time-series states from optimized control inputs
  const Eigen::VectorXd Xex = state_equation_generator_.predict(mpt_mat, Uex);

  // calculate trajectory points from optimization result
  std::vector<TrajectoryPoint> traj_points;
  for (size_t i = 0; i < N_ref; ++i) {
    auto & ref_point = ref_points.at(i);

    const double lat_error = Xex(i * D_x);
    const double yaw_error = Xex(i * D_x + 1);

    // memorize optimization result (optimized_kinematic_state and optimized_input)
    ref_point.optimized_kinematic_state = KinematicState{lat_error, yaw_error};
    if (i == N_ref - 1) {
      ref_point.optimized_input = 0.0;
    } else {
      ref_point.optimized_input = Uex(D_x + i * D_u);
    }

    TrajectoryPoint traj_point;
    traj_point.pose = ref_point.offsetDeviation(lat_error, yaw_error);

    traj_points.push_back(traj_point);
  }

  // NOTE: If generated trajectory's orientation is not so smooth or kinematically infeasible,
  // recalculate orientation here
  // for (size_t i = 0; i < lat_error_vec.size(); ++i) {
  //   auto & ref_point = (i < fixed_ref_points.size())
  //                        ? fixed_ref_points.at(i)
  //                        : non_fixed_ref_points.at(i - fixed_ref_points.size());
  // }
  // motion_utils::insertOrientation(traj_points);

  debug_data_ptr_->toc(__func__, "        ");
  return traj_points;
}

void MPTOptimizer::updateArcLength(std::vector<ReferencePoint> & ref_points) const
{
  for (size_t i = 0; i < ref_points.size(); i++) {
    ref_points.at(i).delta_arc_length =
      (i == 0) ? 0.0 : tier4_autoware_utils::calcDistance2d(ref_points.at(i), ref_points.at(i - 1));
  }
}

void MPTOptimizer::calcExtraPoints(std::vector<ReferencePoint> & ref_points) const
{
  for (size_t i = 0; i < ref_points.size(); ++i) {
    // alpha
    const auto front_wheel_pos =
      trajectory_utils::getNearestPosition(ref_points, i, vehicle_info_.wheel_base_m);

    const bool are_too_close_points =
      tier4_autoware_utils::calcDistance2d(front_wheel_pos, ref_points.at(i).pose.position) < 1e-03;
    const auto front_wheel_yaw =
      are_too_close_points
        ? ref_points.at(i).getYaw()
        : tier4_autoware_utils::calcAzimuthAngle(ref_points.at(i).pose.position, front_wheel_pos);
    ref_points.at(i).alpha =
      tier4_autoware_utils::normalizeRadian(front_wheel_yaw - ref_points.at(i).getYaw());

    /*
    // near objects
    ref_points.at(i).near_objects = [&]() {
      const int avoidance_check_steps =
        mpt_param_.near_objects_length / mpt_param_.delta_arc_length;

      const int avoidance_check_begin_idx =
        std::max(0, static_cast<int>(i) - avoidance_check_steps);
      const int avoidance_check_end_idx =
        std::min(static_cast<int>(ref_points.size()), static_cast<int>(i) + avoidance_check_steps);

      for (int a_idx = avoidance_check_begin_idx; a_idx < avoidance_check_end_idx; ++a_idx) {
        if (ref_points.at(a_idx).vehicle_bounds.at(0).hasCollisionWithObject()) {
          return true;
        }
      }
      return false;
    }();
    */

    // The point are considered to be near the object if nearest previous ref point is near the
    // object.
    if (prev_ref_points_ptr_ && prev_ref_points_ptr_->empty()) {
      const size_t prev_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
        *prev_ref_points_ptr_, tier4_autoware_utils::getPose(ref_points.at(i)),
        traj_param_.delta_dist_threshold_for_closest_point,
        traj_param_.delta_yaw_threshold_for_closest_point);
      const double dist_to_nearest_prev_ref =
        tier4_autoware_utils::calcDistance2d(prev_ref_points_ptr_->at(prev_idx), ref_points.at(i));
      if (dist_to_nearest_prev_ref < 1.0 && prev_ref_points_ptr_->at(prev_idx).near_objects) {
        ref_points.at(i).near_objects = true;
      }
    }
  }
}

void MPTOptimizer::addSteerWeightR(
  std::vector<Eigen::Triplet<double>> & R_triplet_vec,
  const std::vector<ReferencePoint> & ref_points) const
{
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t D_v = D_x + N_u;

  // add steering rate : weight for (u(i) - u(i-1))^2
  for (size_t i = D_x; i < D_v - 1; ++i) {
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i, mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i + 1, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i + 1, mpt_param_.steer_rate_weight));
  }
}

void MPTOptimizer::updateBounds(
  std::vector<ReferencePoint> & ref_points,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound) const
{
  debug_data_ptr_->tic(__func__);

  const double min_soft_road_clearance = vehicle_info_.vehicle_width_m / 2.0;  // TODO(murooka)

  // calculate distance to left/right bound on each reference point
  for (auto & ref_point : ref_points) {
    const double dist_to_left_bound =
      calcLateralDistToBounds(ref_point.pose, left_bound, min_soft_road_clearance, true);
    const double dist_to_right_bound =
      calcLateralDistToBounds(ref_point.pose, right_bound, min_soft_road_clearance, false);
    ref_point.bounds = Bounds{dist_to_right_bound, dist_to_left_bound};
  }

  debug_data_ptr_->toc(__func__, "          ");
  return;
}

void MPTOptimizer::updateVehicleBounds(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  debug_data_ptr_->tic(__func__);

  for (size_t p_idx = 0; p_idx < ref_points.size(); ++p_idx) {
    const auto & ref_point = ref_points.at(p_idx);
    // TODO(murooka) this is required.
    // somtimes vehile_bounds has already value with previous value?
    ref_points.at(p_idx).bounds_on_constraints.clear();
    ref_points.at(p_idx).beta.clear();

    for (const double lon_offset : mpt_param_.vehicle_circle_longitudinal_offsets) {
      const auto collision_check_pose =
        ref_points_spline.getSplineInterpolatedPose(p_idx, lon_offset);
      const double collision_check_yaw = tf2::getYaw(collision_check_pose.orientation);

      // calculate beta
      const double beta = ref_point.getYaw() - collision_check_yaw;
      ref_points.at(p_idx).beta.push_back(beta);

      // calculate vehicle_bounds_pose
      const double tmp_yaw = std::atan2(
        collision_check_pose.position.y - ref_point.pose.position.y,
        collision_check_pose.position.x - ref_point.pose.position.x);
      const double offset_y =
        -tier4_autoware_utils::calcDistance2d(ref_point, collision_check_pose) *
        std::sin(tmp_yaw - collision_check_yaw);

      const auto vehicle_bounds_pose =
        tier4_autoware_utils::calcOffsetPose(collision_check_pose, 0.0, offset_y, 0.0);

      // interpolate bounds
      const auto bounds = [&]() {
        const double collision_check_s = ref_points_spline.getAccumulatedLength(p_idx) + lon_offset;
        const size_t collision_check_idx = ref_points_spline.getOffsetIndex(p_idx, lon_offset);

        const size_t prev_idx = std::clamp(
          collision_check_idx - 1, static_cast<size_t>(0),
          static_cast<size_t>(ref_points_spline.getSize() - 1));
        const size_t next_idx = prev_idx + 1;

        const auto & prev_bounds = ref_points.at(prev_idx).bounds;
        const auto & next_bounds = ref_points.at(next_idx).bounds;

        const double prev_s = ref_points_spline.getAccumulatedLength(prev_idx);
        const double next_s = ref_points_spline.getAccumulatedLength(next_idx);

        // TODO(murooka) is this required?
        const double ratio = std::clamp((collision_check_s - prev_s) / (next_s - prev_s), 0.0, 1.0);

        auto bounds = Bounds::lerp(prev_bounds, next_bounds, ratio);
        bounds.translate(offset_y);
        return bounds;
      }();

      ref_points.at(p_idx).bounds_on_constraints.push_back(bounds);
      ref_points.at(p_idx).pose_on_constraints.push_back(vehicle_bounds_pose);
    }
  }

  debug_data_ptr_->toc(__func__, "          ");
}

void MPTOptimizer::publishDebugTrajectories(
  const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & mpt_traj_points) const
{
  // reference points
  const auto ref_traj = trajectory_utils::createTrajectory(
    header, trajectory_utils::convertToTrajectoryPoints(ref_points));
  debug_ref_traj_pub_->publish(ref_traj);

  // fixed reference points
  const auto fixed_traj_points = extractFixedPoints(ref_points);
  const auto fixed_traj = trajectory_utils::createTrajectory(header, fixed_traj_points);
  debug_fixed_traj_pub_->publish(fixed_traj);

  // mpt points
  const auto mpt_traj = trajectory_utils::createTrajectory(header, mpt_traj_points);
  debug_mpt_traj_pub_->publish(mpt_traj);
}

std::vector<TrajectoryPoint> MPTOptimizer::extractFixedPoints(
  const std::vector<ReferencePoint> & ref_points) const
{
  std::vector<TrajectoryPoint> fixed_traj_points;
  for (const auto & ref_point : ref_points) {
    if (ref_point.fix_kinematic_state) {
      TrajectoryPoint fixed_traj_point;
      fixed_traj_point.pose = ref_point.offsetDeviation(
        ref_point.fix_kinematic_state->lat, ref_point.fix_kinematic_state->yaw);
      fixed_traj_points.push_back(fixed_traj_point);
    }
  }

  return fixed_traj_points;
}
}  // namespace collision_free_path_planner
