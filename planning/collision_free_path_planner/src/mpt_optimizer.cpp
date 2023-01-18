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
#include "collision_free_path_planner/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/motion_utils.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "boost/assign/list_of.hpp"
#include "boost/optional.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
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
  std::vector<double> longitudinal_offsets;
  std::vector<double> radiuses;

  // 1st circle (rear wheel)
  longitudinal_offsets.push_back(0.0);
  radiuses.push_back(vehicle_info.vehicle_width_m / 2.0 * rear_radius_ratio);

  // 2nd circle (front wheel)
  const double radius = std::hypot(
    vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
    vehicle_info.vehicle_width_m / 2.0);

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  const double longitudinal_offset =
    unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_info.rear_overhang_m;

  longitudinal_offsets.push_back(longitudinal_offset);
  radiuses.push_back(radius * front_radius_ratio);

  return {radiuses, longitudinal_offsets};
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> extractBounds(
  const std::vector<ReferencePoint> & ref_points, const size_t l_idx, const double offset)
{
  Eigen::VectorXd ub_vec(ref_points.size());
  Eigen::VectorXd lb_vec(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ub_vec(i) = ref_points.at(i).vehicle_bounds.at(l_idx).upper_bound + offset;
    lb_vec(i) = ref_points.at(i).vehicle_bounds.at(l_idx).lower_bound - offset;
  }
  return {ub_vec, lb_vec};
}

geometry_msgs::msg::Pose offsetPose(
  const ReferencePoint & ref_point, const KinematicState & deviation, const double lon_offset)
{
  const double ref_yaw = ref_point.getYaw();

  geometry_msgs::msg::Pose pose;
  pose.position.x = ref_point.pose.position.x - std::sin(ref_yaw) * deviation.lat -
                    std::cos(ref_yaw + deviation.yaw) * lon_offset;
  pose.position.y = ref_point.pose.position.y + std::cos(ref_yaw) * deviation.lat -
                    std::sin(ref_yaw + deviation.yaw) * lon_offset;

  pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(ref_yaw + deviation.yaw);

  return pose;
}

template <typename T>
void trimPoints(std::vector<T> & points)
{
  std::vector<T> trimmed_points;
  constexpr double epsilon = 1e-6;

  auto itr = points.begin();
  while (itr != points.end() - 1) {
    bool is_overlapping = false;
    if (itr != points.begin()) {
      const auto & p_front = tier4_autoware_utils::getPoint(*itr);
      const auto & p_back = tier4_autoware_utils::getPoint(*(itr + 1));

      const double dx = p_front.x - p_back.x;
      const double dy = p_front.y - p_back.y;
      if (dx * dx + dy * dy < epsilon) {
        is_overlapping = true;
      }
    }
    if (is_overlapping) {
      itr = points.erase(itr);
    } else {
      ++itr;
    }
  }
}

std::vector<double> eigenVectorToStdVector(const Eigen::VectorXd & eigen_vec)
{
  return {eigen_vec.data(), eigen_vec.data() + eigen_vec.rows()};
}

double calcLateralError(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Pose ref_pose)
{
  const double err_x = target_point.x - ref_pose.position.x;
  const double err_y = target_point.y - ref_pose.position.y;
  const double ref_yaw = tf2::getYaw(ref_pose.orientation);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

[[maybe_unused]] Eigen::Vector2d getState(
  const geometry_msgs::msg::Pose & target_pose, const geometry_msgs::msg::Pose & ref_pose)
{
  const double lat_error = calcLateralError(target_pose.position, ref_pose);
  const double yaw_error = tier4_autoware_utils::normalizeRadian(
    tf2::getYaw(target_pose.orientation) - tf2::getYaw(ref_pose.orientation));
  Eigen::VectorXd kinematics = Eigen::VectorXd::Zero(2);
  kinematics << lat_error, yaw_error;
  return kinematics;
}

template <class T>
std::vector<T> createVector(const T & value, const std::vector<T> & vector)
{
  std::vector<T> result_vector;
  result_vector.push_back(value);
  result_vector.insert(result_vector.end(), vector.begin(), vector.end());
  return result_vector;
}

[[maybe_unused]] std::vector<ReferencePoint> resampleRefPoints(
  const std::vector<ReferencePoint> & ref_points, const double interval)
{
  const auto traj_points = trajectory_utils::convertToTrajectoryPoints(ref_points);
  const auto resampled_traj_points =
    trajectory_utils::resampleTrajectoryPoints(traj_points, interval);
  return trajectory_utils::convertToReferencePoints(resampled_traj_points);
}

boost::optional<geometry_msgs::msg::Point> intersect(
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
    return {};
  }

  geometry_msgs::msg::Point intersect_point;
  intersect_point.x = intersect_points.front().x();
  intersect_point.y = intersect_points.front().y();
  return intersect_point;
}

double calcLateralDistToBound(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & bound,
  const double additional_offset, const bool is_left_bound = true)
{
  constexpr double max_lat_offset = 5.0;

  const auto lat_offset_point =
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, max_lat_offset, 0.0).position;
  double dist_to_bound = is_left_bound ? max_lat_offset : -max_lat_offset;

  for (size_t i = 0; i < bound.size() - 1; ++i) {
    const auto intersect_point =
      intersect(pose.position, lat_offset_point, bound.at(i), bound.at(i + 1));
    if (intersect_point) {
      const double tmp_dist =
        tier4_autoware_utils::calcDistance2d(pose.position, *intersect_point) +
        (is_left_bound ? -1 : 1) * additional_offset;
      dist_to_bound = std::min(tmp_dist, dist_to_bound);
    }
  }

  return dist_to_bound;
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

  // vehicle model
  vehicle_model_ptr_ =
    std::make_unique<KinematicsBicycleModel>(vehicle_info_.wheel_base_m, mpt_param_.max_steer_rad);

  // osqp solver
  // osqp_solver_.updateEpsAbs(osqp_epsilon_);
  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(osqp_epsilon_);

  // publisher
  debug_mpt_fixed_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_mpt_ref_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_traj", 1);
}

void MPTOptimizer::initializeMPTParam(
  rclcpp::Node * node, const vehicle_info_util::VehicleInfo & vehicle_info)
{
  mpt_param_ = MPTParam{};

  {  // option
    // TODO(murooka) implement plan_from_ego
    mpt_param_.plan_from_ego = node->declare_parameter<bool>("mpt.option.plan_from_ego");
    mpt_param_.max_plan_from_ego_length =
      node->declare_parameter<double>("mpt.option.max_plan_from_ego_length");
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
    mpt_param_.num_curvature_sampling_points =
      node->declare_parameter<int>("mpt.common.num_curvature_sampling_points");

    mpt_param_.delta_arc_length = node->declare_parameter<double>("mpt.common.delta_arc_length");
  }

  // kinematics
  mpt_param_.max_steer_rad = vehicle_info.max_steer_angle_rad;

  // By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
  // The 0.8 scale is adopted as it performed the best.
  constexpr double default_wheelbase_ratio = 0.8;
  mpt_param_.optimization_center_offset = node->declare_parameter<double>(
    "mpt.kinematics.optimization_center_offset",
    vehicle_info_.wheel_base_m * default_wheelbase_ratio);

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
  prev_ref_points_ptr_ = nullptr;
}

void MPTOptimizer::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option
    updateParam<bool>(parameters, "mpt.option.plan_from_ego", mpt_param_.plan_from_ego);
    updateParam<double>(
      parameters, "mpt.option.max_plan_from_ego_length", mpt_param_.max_plan_from_ego_length);
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
  updateParam<int>(
    parameters, "mpt.common.num_curvature_sampling_points",
    mpt_param_.num_curvature_sampling_points);

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
    } else {
      throw std::invalid_argument(
        "advanced.mpt.collision_free_constraints.vehicle_circles.num parameter is invalid.");
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

boost::optional<MPTTrajs> MPTOptimizer::getModelPredictiveTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points)
{
  debug_data_ptr_->tic(__func__);

  const auto & p = planner_data;
  const auto & path_points = p.path.points;

  // assert arguments
  assert(!smoothed_points.empty());
  assert(!path_points.empty());

  // check if assert is assured
  if (smoothed_points.empty()) {
    logInfo("return boost::none since smoothed_points is empty");
    return boost::none;
  }
  if (path_points.empty()) {
    logInfo("return boost::none since path_points is empty");
    return boost::none;
  }

  // calculate reference points
  std::vector<ReferencePoint> full_ref_points = getReferencePoints(planner_data, smoothed_points);
  if (full_ref_points.empty()) {
    logInfo("return boost::none since ref_points is empty");
    return boost::none;
  } else if (full_ref_points.size() == 1) {
    logInfo("return boost::none since ref_points.size() == 1");
    return boost::none;
  }

  std::vector<ReferencePoint> fixed_ref_points;
  std::vector<ReferencePoint> non_fixed_ref_points;
  bool is_fixing_ref_points = true;
  for (size_t i = 0; i < full_ref_points.size(); ++i) {
    if (i == full_ref_points.size() - 1) {
      if (!full_ref_points.at(i).fix_kinematic_state) {
        is_fixing_ref_points = false;
      }
    } else if (
      // fix first three points
      full_ref_points.at(i).fix_kinematic_state && full_ref_points.at(i + 1).fix_kinematic_state &&
      (i + 2 < full_ref_points.size() && full_ref_points.at(i + 2).fix_kinematic_state) &&
      (i + 3 < full_ref_points.size() && full_ref_points.at(i + 3).fix_kinematic_state)) {
    } else {
      is_fixing_ref_points = false;
    }

    if (is_fixing_ref_points) {
      fixed_ref_points.push_back(full_ref_points.at(i));
    } else {
      non_fixed_ref_points.push_back(full_ref_points.at(i));
    }
  }

  // calculate B and W matrices where x = B u + W
  const auto mpt_matrix = generateMPTMatrix(non_fixed_ref_points);

  // calculate Q and R matrices where J(x, u) = x^t Q x + u^t R u
  const auto value_matrix = generateValueMatrix(non_fixed_ref_points, path_points);

  const auto optimized_control_variables =
    executeOptimization(mpt_matrix, value_matrix, non_fixed_ref_points);
  if (!optimized_control_variables) {
    logInfo("return boost::none since could not solve qp");
    return boost::none;
  }

  const auto mpt_points = getMPTPoints(
    fixed_ref_points, non_fixed_ref_points, optimized_control_variables.get(), mpt_matrix);

  auto full_optimized_ref_points = fixed_ref_points;
  full_optimized_ref_points.insert(
    full_optimized_ref_points.end(), non_fixed_ref_points.begin(), non_fixed_ref_points.end());

  // publish
  publishDebugTrajectories(p.path.header, full_optimized_ref_points, mpt_points);

  debug_data_ptr_->toc(__func__, "      ");

  const auto mpt_trajs = MPTTrajs{full_optimized_ref_points, mpt_points};
  prev_ref_points_ptr_ = std::make_shared<std::vector<ReferencePoint>>(full_optimized_ref_points);

  return mpt_trajs;
}

std::vector<ReferencePoint> MPTOptimizer::getReferencePoints(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points) const
{
  stop_watch_.tic(__func__);

  const auto & p = planner_data;

  const double forward_traj_length = traj_param_.num_sampling_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  // resample and convert smoothed points type from trajectory points to reference points
  auto ref_points = [&]() {
    const auto resampled_smoothed_points =
      trajectory_utils::resampleTrajectoryPoints(smoothed_points, mpt_param_.delta_arc_length);
    return trajectory_utils::convertToReferencePoints(resampled_smoothed_points);
  }();

  // crop reference points with margin first to reduce calculation cost
  const double tmp_margin = 10.0;
  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);
  ref_points = trajectory_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    -backward_traj_length - tmp_margin);

  // calculate spline interpolation of reference points
  SplineInterpolationPoints2d ref_points_spline(ref_points);

  // NOTE: Calculating orientation and curvature requirs points
  // around the target points to be smooth.
  //       Therefore, crop margin is required.
  updateOrientation(ref_points, ref_points_spline);
  updateCurvature(ref_points, ref_points_spline);

  // crop backward
  ref_points = trajectory_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    -backward_traj_length);

  // must be after backward cropping
  calcFixedPoints(ref_points);

  // crop with margin
  /*
  updateBounds(ref_points, p.left_bound, p.right_bound);
  updateVehicleBounds(ref_points, ref_points_spline);

  // set extra information (alpha and has_object_collision)
  // NOTE: This must be after bounds calculation.
  calcExtraPoints(ref_points);
  */

  /*
  // crop forward
  ref_points = trajectory_utils::cropForwardPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length);
  */

  /*
  const auto cropped_smoothed_points =
    [&]() -> std::vector<TrajectoryPoint> {
      const auto resampled_smoothed_points = resampleTrajectoryPoints(smoothed_points,
  mpt_param_.delta_arc_length); const size_t ego_seg_idx =
  findEgoSegmentIndex(resampled_smoothed_points, p.ego_pose, ego_nearest_param_); return
  trajectory_utils::cropBackwardPoints(resampled_smoothed_points, p.ego_pose.position, ego_seg_idx,
  traj_param_.output_backward_traj_length);
    }();

  const auto ref_points = createReferencePoints(cropped_smoothed_points, fixed_ref_points);
  if (ref_points.empty()) {
    return std::vector<ReferencePoint>{};
  }
  */

  // set some information to reference points considering fix kinematics
  // trimPoints(ref_points);
  // calcPlanningFromEgo(
  // p.ego_pose, p.ego_vel,
  // ref_points);  // NOTE: fix_kinematic_state will be updated when planning from ego

  // crop trajectory with margin to calculate vehicle bounds at the end point
  constexpr double tmp_ref_points_margin = 20.0;
  const double ref_length_with_margin =
    traj_param_.num_sampling_points * mpt_param_.delta_arc_length + tmp_ref_points_margin;
  ref_points = trajectory_utils::clipForwardPoints(ref_points, 0, ref_length_with_margin);
  if (ref_points.empty()) {
    return std::vector<ReferencePoint>{};
  }

  // set bounds information
  updateBounds(ref_points, p.left_bound, p.right_bound);
  updateVehicleBounds(ref_points, ref_points_spline);

  // set extra information (alpha and has_object_collision)
  // NOTE: This must be after bounds calculation and updateOrientation
  calcExtraPoints(ref_points);

  const double ref_length = traj_param_.num_sampling_points * mpt_param_.delta_arc_length;
  ref_points = trajectory_utils::clipForwardPoints(ref_points, 0, ref_length);

  // bounds information is assigned to debug data after truncating reference points
  debug_data_ptr_->ref_points = ref_points;

  return ref_points;
  if (ref_points.empty()) {
    return std::vector<ReferencePoint>{};
  }

  debug_data_ptr_->msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return ref_points;
}

void MPTOptimizer::updateOrientation(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  assert(ref_points.size() == ref_points_spline.size());  // NOTE: not assured

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
  assert(ref_points.size() == ref_points_spline.size());  // NOTE: not assured

  const auto curvature_vec = ref_points_spline.getSplineInterpolatedCurvatures();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).k = curvature_vec.at(i);
  }
}

void MPTOptimizer::calcFixedPoints(std::vector<ReferencePoint> & ref_points) const
{
  /*
  // check if planning from ego pose is required
  const bool plan_from_ego = mpt_param_.plan_from_ego && std::abs(ego_vel) < epsilon &&
                             ref_points.size() > 1;
  if (plan_from_ego) {
    return;
  }
  */

  if (prev_ref_points_ptr_) {
    const size_t prev_ref_front_seg_idx = trajectory_utils::findEgoSegmentIndex(
      *prev_ref_points_ptr_, tier4_autoware_utils::getPose(ref_points.front()), ego_nearest_param_);
    const size_t prev_ref_front_point_idx = prev_ref_front_seg_idx;

    // TODO(murooka) check deviation

    const auto front_prev_ref_point =
      prev_ref_points_ptr_->at(prev_ref_front_point_idx);  // velocity is old

    // TODO(murooka) check smoothed_points fron is too close to front_ref_point

    // ref_points = createVector(front_ref_point, ref_points);
    ref_points.front() = front_prev_ref_point;
    ref_points.front().fix_kinematic_state = front_prev_ref_point.optimized_kinematic_state;

    // TODO(murooka) resample reference points since fixed points is not
    // const auto resmapled_ref_points = resampleRefPoints(ref_points,
    // mpt_param_.delta_arc_length); ref_points

    return;
  }

  // No fixed points
  return;
}

/*
void MPTOptimizer::calcPlanningFromEgo(
  const geometry_msgs::msg::Pose & ego_pose, const double ego_vel,
  std::vector<ReferencePoint> & ref_points) const
{
    for (auto & ref_point : ref_points) {
      ref_point.fix_kinematic_state = boost::none;
    }

    // // interpolate and crop backward
    // const auto interpolated_points = interpolation_utils::getInterpolatedPoints(
    //                                                                             points,
    // mpt_param_.delta_arc_length); const auto cropped_interpolated_points =
    // trajectory_utils::clipBackwardPoints( interpolated_points, current_pose_.position,
    // traj_param_.output_backward_traj_length, mpt_param_.delta_arc_length);
    //
    // auto cropped_ref_points =
    //   trajectory_utils::convertToReferencePoints(cropped_interpolated_points);

    // assign fix kinematics
    const size_t nearest_ref_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
      trajectory_utils::convertToPosesWithYawEstimation(trajectory_utils::convertToPoints(ref_points)),
      ego_pose, ego_nearest_param_.dist_threshold, ego_nearest_param_.yaw_threshold);

    // calculate cropped_ref_points.at(nearest_ref_idx) with yaw
    const geometry_msgs::msg::Pose nearest_ref_pose = [&]() -> geometry_msgs::msg::Pose {
      geometry_msgs::msg::Pose pose;
      pose.position = ref_points.at(nearest_ref_idx).p;

      const size_t prev_nearest_ref_idx =
        (nearest_ref_idx == ref_points.size() - 1) ? nearest_ref_idx - 1 : nearest_ref_idx;
      pose.orientation = geometry_utils::getQuaternionFromPoints(
        ref_points.at(prev_nearest_ref_idx + 1).p, ref_points.at(prev_nearest_ref_idx).p);
      return pose;
    }();

    ref_points.at(nearest_ref_idx).fix_kinematic_state = getState(ego_pose, nearest_ref_pose);
    ref_points.at(nearest_ref_idx).plan_from_ego = true;
  }
}
*/

/*
std::vector<ReferencePoint> MPTOptimizer::getFixedReferencePoints(
  const geometry_msgs::msg::Pose & ego_pose, const double ego_vel,
  const std::vector<TrajectoryPoint> & points, const std::shared_ptr<MPTTrajs> prev_trajs) const
{
  if (
    !prev_trajs || prev_trajs->mpt.empty() || prev_trajs->ref_points.empty() ||
    prev_trajs->mpt.size() != prev_trajs->ref_points.size()) {
    return std::vector<ReferencePoint>();
  }

  if (!mpt_param_.fix_points_around_ego) {
    return std::vector<ReferencePoint>();
  }

  const auto & prev_ref_points = prev_trajs->ref_points;
  const int nearest_prev_ref_idx =
    static_cast<int>(motion_utils::findFirstNearestIndexWithSoftConstraints(
      trajectory_utils::convertToPosesWithYawEstimation(trajectory_utils::convertToPoints(prev_ref_points)),
      ego_pose, ego_nearest_param_.dist_threshold, ego_nearest_param_.yaw_threshold));

  // calculate begin_prev_ref_idx
  const int begin_prev_ref_idx = [&]() {
    const int backward_fixing_num =
      traj_param_.output_backward_traj_length / mpt_param_.delta_arc_length;

    return std::max(0, nearest_prev_ref_idx - backward_fixing_num);
  }();

  // calculate end_prev_ref_idx
  const int end_prev_ref_idx = [&]() {
    const double forward_fixed_length = std::max(
      ego_vel * traj_param_.forward_fixing_min_time, traj_param_.forward_fixing_min_distance);

    const int forward_fixing_num =
      forward_fixed_length / mpt_param_.delta_arc_length;
    return std::min(
      static_cast<int>(prev_ref_points.size()) - 1, nearest_prev_ref_idx + forward_fixing_num);
  }();

  bool points_reaching_prev_points_flag = false;
  std::vector<ReferencePoint> fixed_ref_points;
  for (size_t i = begin_prev_ref_idx; i <= static_cast<size_t>(end_prev_ref_idx); ++i) {
    const auto & prev_ref_point = prev_ref_points.at(i);

    if (!points_reaching_prev_points_flag) {
      if (motion_utils::calcSignedArcLength(points, 0, prev_ref_point.p) < 0) {
        continue;
      }
      points_reaching_prev_points_flag = true;
    }

    ReferencePoint fixed_ref_point;
    fixed_ref_point = prev_ref_point;
    fixed_ref_point.fix_kinematic_state = prev_ref_point.optimized_kinematic_state;

    fixed_ref_points.push_back(fixed_ref_point);
    if (mpt_param_.is_fixed_point_single) {
      break;
    }
  }

  return fixed_ref_points;
}
*/

// state equation: x = B u + W (u includes x_0)
// NOTE: Originaly, x_t+1 = Ad x_t + Bd u + Wd.
MPTOptimizer::MPTMatrix MPTOptimizer::generateMPTMatrix(
  const std::vector<ReferencePoint> & ref_points) const
{
  stop_watch_.tic(__func__);

  // NOTE: center offset of vehicle is always 0 in this algorithm.
  // vehicle_model_ptr_->updateCenterOffset(0.0);

  const size_t N_ref = ref_points.size();
  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();
  const size_t D_v = D_x + D_u * (N_ref - 1);

  // matrices for whole state equation
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(D_x * N_ref, D_v);
  Eigen::VectorXd W = Eigen::VectorXd::Zero(D_x * N_ref);

  // matrices for one-step state equation
  Eigen::MatrixXd Ad(D_x, D_x);
  Eigen::MatrixXd Bd(D_x, D_u);
  Eigen::MatrixXd Wd(D_x, 1);

  // calculate one-step state equation considering kinematics N_ref times
  for (size_t i = 0; i < N_ref; ++i) {
    if (i == 0) {
      B.block(0, 0, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);
      continue;
    }

    const int idx_x_i = i * D_x;
    const int idx_x_i_prev = (i - 1) * D_x;
    const int idx_u_i_prev = (i - 1) * D_u;

    // get discrete kinematics matrix A, B, W
    const double ds = ref_points.at(i).delta_arc_length;
    const double ref_k = ref_points.at(std::max(0, static_cast<int>(i) - 1)).k;
    vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, ref_k, ds);

    B.block(idx_x_i, 0, D_x, D_x) = Ad * B.block(idx_x_i_prev, 0, D_x, D_x);
    B.block(idx_x_i, D_x + idx_u_i_prev, D_x, D_u) = Bd;

    for (size_t j = 0; j < i - 1; ++j) {
      size_t idx_u_j = j * D_u;
      B.block(idx_x_i, D_x + idx_u_j, D_x, D_u) =
        Ad * B.block(idx_x_i_prev, D_x + idx_u_j, D_x, D_u);
    }

    W.segment(idx_x_i, D_x) = Ad * W.block(idx_x_i_prev, 0, D_x, 1) + Wd;
  }

  MPTMatrix mpt_mat;
  mpt_mat.B = B;
  mpt_mat.W = W;

  debug_data_ptr_->msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return mpt_mat;
}

// cost function: J = x' Q x + u' R u
MPTOptimizer::ValueMatrix MPTOptimizer::generateValueMatrix(
  const std::vector<ReferencePoint> & ref_points, const std::vector<PathPoint> & path_points) const
{
  // It can be removed
  // if (ref_points.empty()) {
  //   return ValueMatrix{};
  // }

  stop_watch_.tic(__func__);

  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();
  const size_t N_ref = ref_points.size();

  const size_t D_v = D_x + (N_ref - 1) * D_u;

  const bool is_containing_path_terminal_point = trajectory_utils::isNearLastPathPoint(
    ref_points.back(), path_points, 0.0001, traj_param_.delta_yaw_threshold_for_closest_point);

  // update Q
  Eigen::SparseMatrix<double> Q_sparse_mat(D_x * N_ref, D_x * N_ref);
  std::vector<Eigen::Triplet<double>> Q_triplet_vec;
  for (size_t i = 0; i < N_ref; ++i) {
    // TODO(murooka): refactor this
    // this is for plan_from_ego
    // const bool near_kinematic_state_while_planning_from_ego = [&]() {
    //   const size_t min_idx = static_cast<size_t>(std::max(0, static_cast<int>(i) - 20));
    //   const size_t max_idx = std::min(ref_points.size() - 1, i + 20);
    //   for (size_t j = min_idx; j <= max_idx; ++j) {
    //     if (ref_points.at(j).plan_from_ego && ref_points.at(j).fix_kinematic_state) {
    //       return true;
    //     }
    //   }
    //   return false;
    // }();

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

  debug_data_ptr_->msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return m;
}

boost::optional<Eigen::VectorXd> MPTOptimizer::executeOptimization(
  const MPTMatrix & mpt_mat, const ValueMatrix & val_mat,
  const std::vector<ReferencePoint> & ref_points)
{
  if (ref_points.empty()) {
    return Eigen::VectorXd{};
  }

  stop_watch_.tic(__func__);

  const size_t N_ref = ref_points.size();

  // get matrix
  ObjectiveMatrix obj_m = getObjectiveMatrix(mpt_mat, val_mat, ref_points);
  ConstraintMatrix const_m = getConstraintMatrix(mpt_mat, ref_points);

  // manual warm start
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(obj_m.gradient.size());

  if (mpt_param_.enable_manual_warm_start) {
    const size_t D_x = vehicle_model_ptr_->getDimX();

    if (prev_ref_points_ptr_ && 1 < prev_ref_points_ptr_->size()) {
      geometry_msgs::msg::Pose ref_front_point = ref_points.front().pose;

      const size_t seg_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
        *prev_ref_points_ptr_, ref_front_point, ego_nearest_param_.dist_threshold,
        ego_nearest_param_.yaw_threshold);

      u0(0) = prev_ref_points_ptr_->at(seg_idx).optimized_kinematic_state.lat;
      u0(1) = prev_ref_points_ptr_->at(seg_idx).optimized_kinematic_state.yaw;

      // set steer angle
      for (size_t i = 0; i + 1 < N_ref; ++i) {
        const size_t prev_target_idx = std::min(seg_idx + i, prev_ref_points_ptr_->size() - 1);
        u0(D_x + i) = prev_ref_points_ptr_->at(prev_target_idx).optimized_input;
      }
    }
  }

  const Eigen::MatrixXd & H = obj_m.hessian;
  const Eigen::MatrixXd & A = const_m.linear;
  std::vector<double> f;
  std::vector<double> upper_bound;
  std::vector<double> lower_bound;

  if (mpt_param_.enable_manual_warm_start) {
    f = eigenVectorToStdVector(obj_m.gradient + H * u0);
    Eigen::VectorXd A_times_u0 = A * u0;
    upper_bound = eigenVectorToStdVector(const_m.upper_bound - A_times_u0);
    lower_bound = eigenVectorToStdVector(const_m.lower_bound - A_times_u0);
  } else {
    f = eigenVectorToStdVector(obj_m.gradient);
    upper_bound = eigenVectorToStdVector(const_m.upper_bound);
    lower_bound = eigenVectorToStdVector(const_m.lower_bound);
  }

  // initialize or update solver with warm start
  stop_watch_.tic("initOsqp");
  autoware::common::osqp::CSC_Matrix P_csc = autoware::common::osqp::calCSCMatrixTrapezoidal(H);
  autoware::common::osqp::CSC_Matrix A_csc = autoware::common::osqp::calCSCMatrix(A);
  if (mpt_param_.enable_warm_start && prev_mat_n == H.rows() && prev_mat_m == A.rows()) {
    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("mpt_optimizer"), enable_debug_info_, "warm start");

    /*
    osqp_solver_.updateCscP(P_csc);
    osqp_solver_.updateQ(f);
    osqp_solver_.updateCscA(A_csc);
    osqp_solver_.updateL(lower_bound);
    osqp_solver_.updateU(upper_bound);
    */

    osqp_solver_ptr_->updateCscP(P_csc);
    osqp_solver_ptr_->updateQ(f);
    osqp_solver_ptr_->updateCscA(A_csc);
    osqp_solver_ptr_->updateL(lower_bound);
    osqp_solver_ptr_->updateU(upper_bound);

  } else {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("mpt_optimizer"), enable_debug_info_, "no warm start");

    osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
      // obj_m.hessian, const_m.linear, obj_m.gradient, const_m.lower_bound, const_m.upper_bound,
      P_csc, A_csc, f, lower_bound, upper_bound, osqp_epsilon_);
  }
  prev_mat_n = H.rows();
  prev_mat_m = A.rows();

  debug_data_ptr_->msg_stream << "          "
                              << "initOsqp"
                              << ":= " << stop_watch_.toc("initOsqp") << " [ms]\n";

  // solve
  stop_watch_.tic("solveOsqp");
  // const auto result = osqp_solver_.optimize();
  const auto result = osqp_solver_ptr_->optimize();
  debug_data_ptr_->msg_stream << "          "
                              << "solveOsqp"
                              << ":= " << stop_watch_.toc("solveOsqp") << " [ms]\n";

  // check solution status
  const int solution_status = std::get<3>(result);
  if (solution_status != 1) {
    // osqp_solver_.logUnsolvedStatus("[MPT]");
    osqp_solver_ptr_->logUnsolvedStatus("[MPT]");
    return boost::none;
  }

  // print iteration
  const int iteration_status = std::get<4>(result);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("mpt_optimizer"), enable_debug_info_, "iteration: %d", iteration_status);

  // get result
  std::vector<double> result_vec = std::get<0>(result);

  const size_t DIM_U = vehicle_model_ptr_->getDimU();
  const size_t DIM_X = vehicle_model_ptr_->getDimX();
  const Eigen::VectorXd optimized_control_variables =
    Eigen::Map<Eigen::VectorXd>(&result_vec[0], DIM_X + (N_ref - 1) * DIM_U);

  debug_data_ptr_->msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";

  const Eigen::VectorXd optimized_control_variables_with_offset =
    mpt_param_.enable_manual_warm_start
      ? optimized_control_variables + u0.segment(0, DIM_X + (N_ref - 1) * DIM_U)
      : optimized_control_variables;

  return optimized_control_variables_with_offset;
}

MPTOptimizer::ObjectiveMatrix MPTOptimizer::getObjectiveMatrix(
  const MPTMatrix & mpt_mat, const ValueMatrix & val_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  stop_watch_.tic(__func__);

  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();
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

  debug_data_ptr_->msg_stream << "          " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";

  return obj_matrix;
}

// Set constraint: lb <= Ax <= ub
// decision variable
// x := [u0, ..., uN-1 | z00, ..., z0N-1 | z10, ..., z1N-1 | z20, ..., z2N-1]
//   \in \mathbb{R}^{N * (N_vehicle_circle + 1)}
MPTOptimizer::ConstraintMatrix MPTOptimizer::getConstraintMatrix(
  const MPTMatrix & mpt_mat, const std::vector<ReferencePoint> & ref_points) const
{
  stop_watch_.tic(__func__);

  // NOTE: currently, add additional length to soft bounds approximately
  //       for hard bounds
  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();
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
      const double beta = ref_points.at(i).beta.at(l_idx).get();
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

  debug_data_ptr_->msg_stream << "          " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
  return constraint_matrix;
}

std::vector<TrajectoryPoint> MPTOptimizer::getMPTPoints(
  std::vector<ReferencePoint> & fixed_ref_points,
  std::vector<ReferencePoint> & non_fixed_ref_points, const Eigen::VectorXd & Uex,
  const MPTMatrix & mpt_mat)
{
  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();
  const size_t N_ref = static_cast<size_t>(Uex.rows() - D_x) + 1;

  stop_watch_.tic(__func__);

  std::vector<KinematicState> deviation_vec;
  for (const auto & ref_point : fixed_ref_points) {
    deviation_vec.push_back(ref_point.fix_kinematic_state.get());
  }

  const size_t N_kinematic_state = vehicle_model_ptr_->getDimX();
  const Eigen::VectorXd Xex = mpt_mat.B * Uex + mpt_mat.W;

  for (size_t i = 0; i < non_fixed_ref_points.size(); ++i) {
    const double lat = Xex(i * N_kinematic_state);
    const double yaw = Xex(i * N_kinematic_state + 1);
    deviation_vec.push_back(KinematicState{lat, yaw});
  }

  // calculate trajectory from optimization result
  std::vector<TrajectoryPoint> traj_points;
  debug_data_ptr_->vehicle_circles_pose.resize(deviation_vec.size());
  for (size_t i = 0; i < deviation_vec.size(); ++i) {
    auto & ref_point = (i < fixed_ref_points.size())
                         ? fixed_ref_points.at(i)
                         : non_fixed_ref_points.at(i - fixed_ref_points.size());
    const auto & deviation = deviation_vec.at(i);
    const double lat_error = deviation.lat;
    const double yaw_error = deviation.yaw;

    geometry_msgs::msg::Pose ref_pose = ref_point.pose;
    debug_data_ptr_->mpt_ref_poses.push_back(ref_pose);
    debug_data_ptr_->lateral_errors.push_back(lat_error);

    ref_point.optimized_kinematic_state = KinematicState{lat_error, yaw_error};
    if (fixed_ref_points.size() <= i) {
      const size_t j = i - fixed_ref_points.size();
      if (j == N_ref - 1) {
        ref_point.optimized_input = 0.0;
      } else {
        ref_point.optimized_input = Uex(D_x + j * D_u);
      }
    }

    TrajectoryPoint traj_point;
    traj_point.pose = offsetPose(ref_point, deviation, 0.0);

    traj_points.push_back(traj_point);

    {  // for debug visualization
      const double ref_yaw = ref_point.getYaw();

      const double base_x = ref_point.pose.position.x - std::sin(ref_yaw) * lat_error;
      const double base_y = ref_point.pose.position.y + std::cos(ref_yaw) * lat_error;

      // NOTE: coordinate of vehicle_circle_longitudinal_offsets is back wheel center
      for (const double d : mpt_param_.vehicle_circle_longitudinal_offsets) {
        geometry_msgs::msg::Pose vehicle_circle_pose;

        vehicle_circle_pose.position.x = base_x + d * std::cos(ref_yaw + yaw_error);
        vehicle_circle_pose.position.y = base_y + d * std::sin(ref_yaw + yaw_error);

        vehicle_circle_pose.orientation =
          tier4_autoware_utils::createQuaternionFromYaw(ref_yaw + ref_point.alpha);

        debug_data_ptr_->vehicle_circles_pose.at(i).push_back(vehicle_circle_pose);
      }
    }
  }

  // NOTE: If generated trajectory's orientation is not so smooth or kinematically infeasible,
  // recalculate orientation here for (size_t i = 0; i < lat_error_vec.size(); ++i) {
  //   auto & ref_point = (i < fixed_ref_points.size())
  //                        ? fixed_ref_points.at(i)
  //                        : non_fixed_ref_points.at(i - fixed_ref_points.size());
  //
  //   if (i > 0 && traj_points.size() > 1) {
  //     traj_points.at(i).pose.orientation = geometry_utils::getQuaternionFromPoints(
  //       traj_points.at(i).pose.position, traj_points.at(i - 1).pose.position);
  //   } else if (traj_points.size() > 1) {
  //     traj_points.at(i).pose.orientation = geometry_utils::getQuaternionFromPoints(
  //       traj_points.at(i + 1).pose.position, traj_points.at(i).pose.position);
  //   } else {
  //     traj_points.at(i).pose.orientation =
  //       tier4_autoware_utils::createQuaternionFromYaw(ref_point.yaw);
  //   }
  // }

  debug_data_ptr_->msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";

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
  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();
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
  debug_data_ptr_->sequential_bounds.clear();
  for (auto & ref_point : ref_points) {
    const double dist_to_left_bound =
      calcLateralDistToBound(ref_point.pose, left_bound, min_soft_road_clearance, true);
    const double dist_to_right_bound =
      calcLateralDistToBound(ref_point.pose, left_bound, min_soft_road_clearance, false);
    ref_point.bounds = Bounds{dist_to_right_bound, dist_to_left_bound};
    debug_data_ptr_->sequential_bounds.push_back(ref_point.bounds);
  }

  debug_data_ptr_->toc(__func__, "          ");
  return;
}

void MPTOptimizer::updateVehicleBounds(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  stop_watch_.tic(__func__);

  if (ref_points.size() == 1) {
    for ([[maybe_unused]] const double d : mpt_param_.vehicle_circle_longitudinal_offsets) {
      ref_points.at(0).vehicle_bounds.push_back(ref_points.at(0).bounds);
      ref_points.at(0).beta.push_back(0.0);
    }
    return;
  }

  for (size_t p_idx = 0; p_idx < ref_points.size(); ++p_idx) {
    const auto & ref_point = ref_points.at(p_idx);
    /*
    // TODO(murooka) remove this
    ref_points.at(p_idx).vehicle_bounds.clear();
    ref_points.at(p_idx).beta.clear();
    */

    for (const double lon_offset : mpt_param_.vehicle_circle_longitudinal_offsets) {
      geometry_msgs::msg::Pose collision_check_pose;
      collision_check_pose.position =
        ref_points_spline.getSplineInterpolatedPoint(p_idx, lon_offset);
      const double collision_check_yaw =
        ref_points_spline.getSplineInterpolatedYaw(p_idx, lon_offset);
      collision_check_pose.orientation =
        tier4_autoware_utils::createQuaternionFromYaw(collision_check_yaw);

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
          static_cast<size_t>(ref_points_spline.getSize() - 2));
        const size_t next_idx = prev_idx + 1;

        const auto prev_bounds = ref_points.at(prev_idx).bounds;
        const auto next_bounds = ref_points.at(next_idx).bounds;

        const double prev_s = ref_points_spline.getAccumulatedLength(prev_idx);
        const double next_s = ref_points_spline.getAccumulatedLength(next_idx);
        const double ratio = (collision_check_s - prev_s) / (next_s - prev_s);

        auto bounds = Bounds::lerp(prev_bounds, next_bounds, ratio);
        bounds.translate(offset_y);
        return bounds;
      }();

      ref_points.at(p_idx).vehicle_bounds.push_back(bounds);
      ref_points.at(p_idx).vehicle_bounds_poses.push_back(vehicle_bounds_pose);
      /*
      for (size_t r_idx = 0; r_idx < ref_points.size(); ++r_idx) {
        const double current_s = ref_points_spline.getAccumulatedLength(r_idx);
        if (collision_check_s <= current_s) {
          double prev_avoid_idx;
          if (r_idx == 0) {
            prev_avoid_idx = r_idx;
          } else {
            prev_avoid_idx = r_idx - 1;
          }

          const double prev_s =
            ref_points_spline.getAccumulatedLength(prev_avoid_idx);
          const double next_s =
            ref_points_spline.getAccumulatedLength(prev_avoid_idx + 1);
          const double ratio = (collision_check_s - prev_s) / (next_s - prev_s);

          const auto prev_bounds = ref_points.at(prev_avoid_idx).bounds;
          const auto next_bounds = ref_points.at(prev_avoid_idx + 1).bounds;

          auto bounds = Bounds::lerp(prev_bounds, next_bounds, ratio);
          bounds.translate(offset_y);

          ref_points.at(p_idx).vehicle_bounds.push_back(bounds);
          break;
        }

        if (r_idx == ref_points.size() - 1) {
          ref_points.at(p_idx).vehicle_bounds.push_back(ref_points.back().bounds);
        }
      }
      */
    }
  }

  debug_data_ptr_->msg_stream << "          " << __func__ << ":= " << stop_watch_.toc(__func__)
                              << " [ms]\n";
}

// functions for debug publish
void MPTOptimizer::publishDebugTrajectories(
  const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & mpt_traj_points) const
{
  const auto mpt_fixed_traj_points = extractMPTFixedPoints(ref_points);
  const auto mpt_fixed_traj = trajectory_utils::createTrajectory(header, mpt_fixed_traj_points);
  debug_mpt_fixed_traj_pub_->publish(mpt_fixed_traj);

  const auto ref_traj = trajectory_utils::createTrajectory(
    header, trajectory_utils::convertToTrajectoryPoints(ref_points));
  debug_mpt_ref_traj_pub_->publish(ref_traj);

  const auto mpt_traj = trajectory_utils::createTrajectory(header, mpt_traj_points);
  debug_mpt_traj_pub_->publish(mpt_traj);
}

std::vector<TrajectoryPoint> MPTOptimizer::extractMPTFixedPoints(
  const std::vector<ReferencePoint> & ref_points) const
{
  std::vector<TrajectoryPoint> mpt_fixed_traj;
  for (const auto & ref_point : ref_points) {
    if (ref_point.fix_kinematic_state) {
      TrajectoryPoint fixed_traj_point;
      fixed_traj_point.pose = offsetPose(ref_point, ref_point.fix_kinematic_state.get(), 0.0);
      mpt_fixed_traj.push_back(fixed_traj_point);
    }
  }

  return mpt_fixed_traj;
}
}  // namespace collision_free_path_planner
