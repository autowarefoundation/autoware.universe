// Copyright 2020 Tier IV, Inc.
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

#include <chrono>

#include "boost/optional.hpp"

#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2/utils.h"

#include "opencv2/core.hpp"

#include "osqp_interface/osqp_interface.hpp"

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "obstacle_avoidance_planner/process_cv.hpp"
#include "obstacle_avoidance_planner/util.hpp"
#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

MPTOptimizer::MPTOptimizer(
  const bool is_showing_debug_info, const QPParam & qp_param, const TrajectoryParam & traj_param,
  const ConstrainParam & constraint_param, const VehicleParam & vehicle_param,
  const MPTParam & mpt_param)
: is_showing_debug_info_(is_showing_debug_info)
{
  qp_param_ptr_ = std::make_unique<QPParam>(qp_param);
  traj_param_ptr_ = std::make_unique<TrajectoryParam>(traj_param);
  constraint_param_ptr_ = std::make_unique<ConstrainParam>(constraint_param);
  vehicle_param_ptr_ = std::make_unique<VehicleParam>(vehicle_param);
  mpt_param_ptr_ = std::make_unique<MPTParam>(mpt_param);

  vehicle_model_ptr_ = std::make_unique<KinematicsBicycleModel>(
    vehicle_param_ptr_->wheelbase, vehicle_param_ptr_->max_steer_rad,
    vehicle_param_ptr_->steer_tau);
}

MPTOptimizer::~MPTOptimizer() {}

boost::optional<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>>
MPTOptimizer::getModelPredictiveTrajectory(
  const bool enable_avoidance,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_points,
  const std::vector<autoware_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs, const CVMaps & maps,
  const geometry_msgs::msg::Pose & ego_pose, DebugData * debug_data)
{
  auto t_start1 = std::chrono::high_resolution_clock::now();
  if (smoothed_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger(
        "MPTOptimizer"), is_showing_debug_info_,
      "return boost::none since smoothed_points is empty");
    return boost::none;
  }

  geometry_msgs::msg::Pose origin_pose = smoothed_points.front().pose;
  if (prev_trajs) {
    const int prev_nearest_idx = util::getNearestIdx(
      prev_trajs->model_predictive_trajectory, smoothed_points.front().pose.position);
    origin_pose = prev_trajs->model_predictive_trajectory.at(prev_nearest_idx).pose;
  }
  std::vector<ReferencePoint> ref_points =
    getReferencePoints(origin_pose, ego_pose, smoothed_points, prev_trajs, debug_data);
  if (ref_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger(
        "MPTOptimizer"), is_showing_debug_info_, "return boost::none since ref_points is empty");
    return boost::none;
  }

  const auto mpt_matrix = generateMPTMatrix(ref_points, path_points);
  if (!mpt_matrix) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger(
        "MPTOptimizer"), is_showing_debug_info_, "return boost::none since matrix has nan");
    return boost::none;
  }
  const auto initial_state = getInitialState(origin_pose, ref_points.front());

  const auto optimized_control_variables = executeOptimization(
    enable_avoidance, mpt_matrix.get(), ref_points, path_points, maps, initial_state, debug_data);
  if (!optimized_control_variables) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger(
        "MPTOptimizer"), is_showing_debug_info_, "return boost::none since could not solve qp");
    return boost::none;
  }

  const auto mpt_points = getMPTPoints(
    ref_points, optimized_control_variables.get(), mpt_matrix.get(), initial_state,
    smoothed_points);

  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger(
      "MPTOptimizer"), is_showing_debug_info_, "MPT time: = %f [ms]", elapsed_ms1);
  return mpt_points;
}

std::vector<ReferencePoint> MPTOptimizer::getReferencePoints(
  const geometry_msgs::msg::Pose & origin_pose, const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const std::unique_ptr<Trajectories> & prev_trajs, DebugData * debug_data) const
{
  const auto ref_points = convertToReferencePoints(points, ego_pose, prev_trajs, debug_data);

  const int begin_idx = util::getNearestPointIdx(ref_points, origin_pose.position);
  const auto first_it = ref_points.begin() + begin_idx;
  const int num_points =
    std::min((int)ref_points.size() - 1 - begin_idx, traj_param_ptr_->num_sampling_points);
  return std::vector<ReferencePoint>(first_it, first_it + num_points);
}

std::vector<ReferencePoint> MPTOptimizer::convertToReferencePoints(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & ego_pose, const std::unique_ptr<Trajectories> & prev_trajs,
  DebugData * debug_data) const
{
  const auto interpolated_points =
    util::getInterpolatedPoints(points, traj_param_ptr_->delta_arc_length_for_mpt_points);
  if (interpolated_points.empty()) {
    return std::vector<ReferencePoint>{};
  }

  auto reference_points = getBaseReferencePoints(interpolated_points, points);

  calcCurvature(&reference_points);
  calcArcLength(&reference_points);
  calcExtraPoints(&reference_points);
  calcFixPoints(prev_trajs, ego_pose, &reference_points, debug_data);
  return reference_points;
}

void MPTOptimizer::calcCurvature(std::vector<ReferencePoint> * ref_points) const
{
  if (!ref_points) {
    return;
  }

  int num_points = static_cast<int>(ref_points->size());

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  int max_smoothing_num = static_cast<int>(std::floor(0.5 * (num_points - 1)));
  int L = std::min(mpt_param_ptr_->num_curvature_sampling_points, max_smoothing_num);
  for (int i = L; i < num_points - L; ++i) {
    p1 = ref_points->at(i - L).p;
    p2 = ref_points->at(i).p;
    p3 = ref_points->at(i + L).p;
    double den = std::max(
      util::calculate2DDistance(p1, p2) * util::calculate2DDistance(p2, p3) *
      util::calculate2DDistance(p3, p1),
      0.0001);
    const double curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    ref_points->at(i).k = curvature;
  }

  /* first and last curvature is copied from next value */
  for (int i = 0; i < std::min(L, num_points); ++i) {
    ref_points->at(i).k = ref_points->at(std::min(L, num_points - 1)).k;
    ref_points->at(num_points - i - 1).k = ref_points->at(std::max(num_points - L - 1, 0)).k;
  }
}

void MPTOptimizer::calcArcLength(std::vector<ReferencePoint> * ref_points) const
{
  if (!ref_points) {
    return;
  }
  for (int i = 0; i < ref_points->size(); i++) {
    if (i > 0) {
      geometry_msgs::msg::Point a, b;
      a = ref_points->at(i).p;
      b = ref_points->at(i - 1).p;
      ref_points->at(i).s = ref_points->at(i - 1).s + util::calculate2DDistance(a, b);
    }
  }
}

void MPTOptimizer::calcExtraPoints(std::vector<ReferencePoint> * ref_points) const
{
  if (!ref_points) {
    return;
  }
  for (int i = 0; i < ref_points->size(); i++) {
    const double p1_target_s = ref_points->at(i).s + mpt_param_ptr_->top_point_dist_from_base_link;
    const int nearest_p1_idx = util::getNearestIdx(*ref_points, p1_target_s, i);
    ref_points->at(i).top_pose.position = ref_points->at(nearest_p1_idx).p;

    const double p2_target_s = ref_points->at(i).s + mpt_param_ptr_->mid_point_dist_from_base_link;
    const int nearest_p2_idx = util::getNearestIdx(*ref_points, p2_target_s, i);
    ref_points->at(i).mid_pose.position = ref_points->at(nearest_p2_idx).p;

    ref_points->at(i).top_pose.orientation =
      util::getQuaternionFromPoints(ref_points->at(i).top_pose.position, ref_points->at(i).p);
    if (i == nearest_p1_idx && i > 0) {
      ref_points->at(i).top_pose.orientation =
        util::getQuaternionFromPoints(ref_points->at(i).p, ref_points->at(i - 1).p);
    } else if (i == nearest_p1_idx) {
      ref_points->at(i).top_pose.orientation = ref_points->at(i).q;
    }
    ref_points->at(i).mid_pose.orientation = ref_points->at(i).top_pose.orientation;

    const double delta_yaw =
      tf2::getYaw(ref_points->at(i).top_pose.orientation) - ref_points->at(i).yaw;
    const double norm_delta_yaw = util::normalizeRadian(delta_yaw);
    ref_points->at(i).delta_yaw_from_p1 = norm_delta_yaw;
    ref_points->at(i).delta_yaw_from_p2 = ref_points->at(i).delta_yaw_from_p1;
  }
}

void MPTOptimizer::calcFixPoints(
  const std::unique_ptr<Trajectories> & prev_trajs, const geometry_msgs::msg::Pose & ego_pose,
  std::vector<ReferencePoint> * ref_points, DebugData * debug_data) const
{
  if (!ref_points) {
    return;
  }

  const int nearest_idx_from_ego = util::getNearestPointIdx(*ref_points, ego_pose.position);
  auto t_start1 = std::chrono::high_resolution_clock::now();
  constexpr double fine_resolution = 0.005;
  std::vector<geometry_msgs::msg::Point> fine_interpolated_points;
  if (prev_trajs) {
    fine_interpolated_points =
      util::getInterpolatedPoints(prev_trajs->model_predictive_trajectory, fine_resolution);
  } else {
    fine_interpolated_points = util::getInterpolatedPoints(*ref_points, fine_resolution);
  }
  if (fine_interpolated_points.empty()) {
    return;
  }
  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger(
      "MPTOptimizer"), is_showing_debug_info_, "fine interpo time: = %f [ms]", elapsed_ms1);

  for (int i = 0; i < ref_points->size(); i++) {
    if (
      i >= nearest_idx_from_ego - (int)traj_param_ptr_->num_fix_points_for_mpt / 2 &&
      i < nearest_idx_from_ego + (int)traj_param_ptr_->num_fix_points_for_mpt / 2)
    {
      ref_points->at(i).is_fix = true;
      const int nearest_idx = util::getNearestIdx(fine_interpolated_points, ref_points->at(i).p);
      ref_points->at(i).fixing_lat =
        calcLateralError(fine_interpolated_points[nearest_idx], ref_points->at(i));

      geometry_msgs::msg::Pose debug_pose;
      geometry_msgs::msg::Point rel_point;
      rel_point.y = ref_points->at(i).fixing_lat;
      geometry_msgs::msg::Pose origin;
      origin.position = ref_points->at(i).p;
      origin.orientation = ref_points->at(i).q;
      debug_pose.position = util::transformToAbsoluteCoordinate2D(rel_point, origin);
      debug_data->fixed_mpt_points.push_back(debug_pose);
    }
  }
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
boost::optional<MPTMatrix> MPTOptimizer::generateMPTMatrix(
  const std::vector<ReferencePoint> & reference_points,
  const std::vector<autoware_planning_msgs::msg::PathPoint> & path_points) const
{
  const int N = reference_points.size();
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_X);      //state transition
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_U * N);  // control input
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(DIM_X * N, 1);          //
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  Eigen::MatrixXd R1ex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd R2ex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd Urefex = Eigen::MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Eigen::MatrixXd Q_adaptive = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpt_param_ptr_->lat_error_weight;
  Q(1, 1) = mpt_param_ptr_->yaw_error_weight;
  R(0, 0) = mpt_param_ptr_->steer_input_weight;

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  geometry_msgs::msg::Pose last_ref_pose;
  last_ref_pose.position = reference_points.back().p;
  last_ref_pose.orientation = reference_points.back().q;
  const auto last_extended_point = util::getLastExtendedPoint(
    path_points.back(), last_ref_pose, traj_param_ptr_->delta_yaw_threshold_for_closest_point,
    traj_param_ptr_->max_dist_for_extending_end_point);

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i) {
    const double ref_k = reference_points[i].k;
    double ds = 0.0;
    if (i > 0) {
      ds = reference_points[i].s - reference_points[i - 1].s;
    }

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(&Ad, &Bd, &Cd, &Wd, ds);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1 && last_extended_point) {
      Q_adaptive(0, 0) = mpt_param_ptr_->terminal_path_lat_error_weight;
      Q_adaptive(1, 1) = mpt_param_ptr_->terminal_path_yaw_error_weight;
    } else if (i == N - 1) {
      Q_adaptive(0, 0) = mpt_param_ptr_->terminal_lat_error_weight;
      Q_adaptive(1, 1) = mpt_param_ptr_->terminal_yaw_error_weight;
    }

    /* update mpt matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->calculateReferenceInput(&Uref);
    if (std::fabs(Uref(0, 0)) < mpt_param_ptr_->zero_ff_steer_angle * M_PI / 180.0) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  addSteerWeightR(&R1ex, reference_points);

  MPTMatrix m;
  m.Aex = Aex;
  m.Bex = Bex;
  m.Wex = Wex;
  m.Cex = Cex;
  m.Qex = Qex;
  m.R1ex = R1ex;
  m.R2ex = R2ex;
  m.Urefex = Urefex;
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Urefex.array().isNaN().any())
  {
    RCLCPP_WARN(rclcpp::get_logger("MPTOptimizer"), "[Avoidance] MPT matrix includes NaN.");
    return boost::none;
  }
  return m;
}

void MPTOptimizer::addSteerWeightR(
  Eigen::MatrixXd * R, const std::vector<ReferencePoint> & ref_points) const
{
  const int N = ref_points.size();
  constexpr double DT = 0.1;
  constexpr double ctrl_period = 0.03;

  /* add steering rate : weight for (u(i) - u(i-1) / dt )^2 */
  for (int i = 0; i < N - 1; ++i) {
    const double steer_rate_r = mpt_param_ptr_->steer_rate_weight / (DT * DT);
    (*R)(i + 0, i + 0) += steer_rate_r;
    (*R)(i + 1, i + 0) -= steer_rate_r;
    (*R)(i + 0, i + 1) -= steer_rate_r;
    (*R)(i + 1, i + 1) += steer_rate_r;
  }
  if (N > 1) {
    // steer rate i = 0
    (*R)(0, 0) += mpt_param_ptr_->steer_rate_weight / (ctrl_period * ctrl_period);
  }

  /* add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2 */
  const double steer_acc_r = mpt_param_ptr_->steer_acc_weight / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 3) * ctrl_period);
  const double steer_acc_r_cp2 =
    mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 2) * std::pow(ctrl_period, 2));
  const double steer_acc_r_cp4 = mpt_param_ptr_->steer_acc_weight / std::pow(ctrl_period, 4);
  for (int i = 1; i < N - 1; ++i) {
    (*R)(i - 1, i - 1) += (steer_acc_r);
    (*R)(i - 1, i + 0) += (steer_acc_r * -2.0);
    (*R)(i - 1, i + 1) += (steer_acc_r);
    (*R)(i + 0, i - 1) += (steer_acc_r * -2.0);
    (*R)(i + 0, i + 0) += (steer_acc_r * 4.0);
    (*R)(i + 0, i + 1) += (steer_acc_r * -2.0);
    (*R)(i + 1, i - 1) += (steer_acc_r);
    (*R)(i + 1, i + 0) += (steer_acc_r * -2.0);
    (*R)(i + 1, i + 1) += (steer_acc_r);
  }
  if (N > 1) {
    // steer acc i = 1
    (*R)(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
    (*R)(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
    (*R)(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
    (*R)(1, 1) += steer_acc_r * 1.0;
    // steer acc i = 0
    (*R)(0, 0) += steer_acc_r_cp4 * 1.0;
  }
}

void MPTOptimizer::addSteerWeightF(Eigen::VectorXd * f) const
{
  constexpr double DT = 0.1;
  constexpr double ctrl_period = 0.03;
  constexpr double raw_steer_cmd_prev = 0;
  constexpr double raw_steer_cmd_pprev = 0;

  if (f->rows() < 2) {
    return;
  }

  // steer rate for i = 0
  (*f)(0) += -2.0 * mpt_param_ptr_->steer_rate_weight / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = mpt_param_.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 3) * ctrl_period);
  const double steer_acc_r_cp2 =
    mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 2) * std::pow(ctrl_period, 2));
  const double steer_acc_r_cp4 = mpt_param_ptr_->steer_acc_weight / std::pow(ctrl_period, 4);

  // steer acc  i = 0
  (*f)(0) += ((-2.0 * raw_steer_cmd_prev + raw_steer_cmd_pprev) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  (*f)(0) += (-2.0 * raw_steer_cmd_prev * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  (*f)(1) += (2.0 * raw_steer_cmd_prev * steer_acc_r_cp1) * 0.5;
}

boost::optional<Eigen::VectorXd> MPTOptimizer::executeOptimization(
  const bool enable_avoidance, const MPTMatrix & m, const std::vector<ReferencePoint> & ref_points,
  const std::vector<autoware_planning_msgs::msg::PathPoint> & path_points, const CVMaps & maps,
  const Eigen::VectorXd & x0, DebugData * debug_data)
{
  auto t_start1 = std::chrono::high_resolution_clock::now();

  ObjectiveMatrix obj_m = getObjectiveMatrix(x0, m);
  ConstraintMatrix const_m =
    getConstraintMatrix(enable_avoidance, x0, m, maps, ref_points, path_points, debug_data);

  osqp_solver_ptr_ = std::make_unique<osqp::OSQPInterface>(
    obj_m.hessian, const_m.linear, obj_m.gradient, const_m.lower_bound, const_m.upper_bound,
    1.0e-3);
  osqp_solver_ptr_->updateEpsRel(1.0e-3);
  const auto result = osqp_solver_ptr_->optimize();

  int solution_status = std::get<3>(result);
  if (solution_status != 1) {
    util::logOSQPSolutionStatus(solution_status);
    return boost::none;
  }

  std::vector<double> result_vec = std::get<0>(result);
  const Eigen::VectorXd optimized_control_variables =
    Eigen::Map<Eigen::VectorXd>(&result_vec[0], ref_points.size());

  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger(
      "MPTOptimizer"), is_showing_debug_info_, "mpt opt time: = %f [ms]", elapsed_ms1);
  return optimized_control_variables;
}

double MPTOptimizer::calcLateralError(
  const geometry_msgs::msg::Point & target_point, const ReferencePoint & ref_point) const
{
  const double err_x = target_point.x - ref_point.p.x;
  const double err_y = target_point.y - ref_point.p.y;
  const double ref_yaw = tf2::getYaw(ref_point.q);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

Eigen::VectorXd MPTOptimizer::getInitialState(
  const geometry_msgs::msg::Pose & target_pose, const ReferencePoint & nearest_ref_point) const
{
  const double lat_error = calcLateralError(target_pose.position, nearest_ref_point);
  const double yaw_error =
    util::normalizeRadian(tf2::getYaw(target_pose.orientation) - nearest_ref_point.yaw);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
  x0 << lat_error, yaw_error, vehicle_param_ptr_->wheelbase * nearest_ref_point.k;
  return x0;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> MPTOptimizer::getMPTPoints(
  const std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & Uex,
  const MPTMatrix & mpt_matrix, const Eigen::VectorXd & x0,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & optimized_points) const
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  Eigen::VectorXd Xex = mpt_matrix.Aex * x0 + mpt_matrix.Bex * Uex + mpt_matrix.Wex;

  const int N = ref_points.size();

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj_points;
  for (int i = 0; i < ref_points.size(); ++i) {
    const double lat_error = Xex(i * DIM_X);
    const double yaw_error = Xex(i * DIM_X + 1);
    autoware_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose.position.x = ref_points[i].p.x - std::sin(ref_points[i].yaw) * lat_error;
    traj_point.pose.position.y = ref_points[i].p.y + std::cos(ref_points[i].yaw) * lat_error;
    traj_point.twist.linear.x = ref_points[i].v;

    traj_points.push_back(traj_point);
  }
  for (int i = 0; i < traj_points.size(); ++i) {
    if (i > 0 && traj_points.size() > 1) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    } else if (traj_points.size() > 1) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    } else {
      traj_points[i].pose.orientation = ref_points[i].q;
    }
  }
  return traj_points;
}

std::vector<Bounds> MPTOptimizer::getReferenceBounds(
  const bool enable_avoidance, const std::vector<ReferencePoint> & ref_points, const CVMaps & maps,
  DebugData * debug_data) const
{
  std::vector<Bounds> ref_bounds;
  std::vector<geometry_msgs::msg::Pose> debug_bounds_candidata_for_base_points;
  std::vector<geometry_msgs::msg::Pose> debug_bounds_candidata_for_top_points;
  int cnt = 0;
  for (const auto & point : ref_points) {
    ReferencePoint ref_base_point;
    ref_base_point.p = point.p;
    ref_base_point.yaw = point.yaw;

    ReferencePoint ref_top_point;
    ref_top_point.p = point.top_pose.position;
    ref_top_point.yaw = tf2::getYaw(point.top_pose.orientation);

    ReferencePoint ref_mid_point;
    ref_mid_point.p = point.mid_pose.position;
    ref_mid_point.yaw = tf2::getYaw(point.mid_pose.orientation);

    geometry_msgs::msg::Pose debug_for_base_point;
    debug_for_base_point.position = ref_base_point.p;
    debug_for_base_point.orientation = point.q;
    debug_bounds_candidata_for_base_points.push_back(debug_for_base_point);

    geometry_msgs::msg::Pose debug_for_top_point;
    debug_for_top_point.position = ref_top_point.p;
    debug_for_top_point.orientation = point.top_pose.orientation;
    debug_bounds_candidata_for_top_points.push_back(debug_for_top_point);

    // Calculate boundaries.
    auto lat_bounds_0 = getBound(enable_avoidance, ref_base_point, maps);
    auto lat_bounds_1 = getBound(enable_avoidance, ref_top_point, maps);
    auto lat_bounds_2 = getBound(enable_avoidance, ref_mid_point, maps);
    if (
      lat_bounds_0[0] == lat_bounds_0[1] || lat_bounds_1[0] == lat_bounds_1[1] ||
      lat_bounds_2[0] == lat_bounds_2[1])
    {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("MPTOptimizer"),
        clock,
        std::chrono::milliseconds(1000).count(),
        "[Avoidance] Could not find driveable area for %i th point", cnt);
      RCLCPP_INFO_EXPRESSION(
        rclcpp::get_logger(
          "MPTOptimizer"), is_showing_debug_info_, "Path is blocked at %i ", cnt);
      Bounds bounds;
      bounds.c0 = {1, -1};
      bounds.c1 = {1, -1};
      bounds.c2 = {1, -1};
      ref_bounds.emplace_back(bounds);
      cnt++;
      continue;
    }
    Bounds bounds;
    bounds.c0 = lat_bounds_0;
    bounds.c1 = lat_bounds_1;
    bounds.c2 = lat_bounds_2;
    ref_bounds.emplace_back(bounds);
    cnt++;
  }
  debug_data->bounds = ref_bounds;
  debug_data->bounds_candidate_for_base_points = debug_bounds_candidata_for_base_points;
  debug_data->bounds_candidate_for_top_points = debug_bounds_candidata_for_top_points;
  return ref_bounds;
}

std::vector<double> MPTOptimizer::getBound(
  const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps) const
{
  double left_bound = 0;
  double right_bound = 0;
  const double left_angle = util::normalizeRadian(ref_point.yaw + M_PI_2);
  const double right_angle = util::normalizeRadian(ref_point.yaw - M_PI_2);

  geometry_msgs::msg::Point new_position;
  new_position.x = ref_point.p.x;
  new_position.y = ref_point.p.y;
  double original_clearance = getClearance(maps.clearance_map, new_position, maps.map_info);
  double original_object_clearance =
    getClearance(maps.only_objects_clearance_map, new_position, maps.map_info);
  if (
    original_clearance > mpt_param_ptr_->clearance_from_road &&
    original_object_clearance > mpt_param_ptr_->clearance_from_object)
  {
    const double initial_dist = 0;
    right_bound =
      -1 * getTraversedDistance(enable_avoidance, ref_point, right_angle, initial_dist, maps);
    left_bound = getTraversedDistance(enable_avoidance, ref_point, left_angle, initial_dist, maps);
  } else {
    const double initial_dist = 0;
    const bool search_expanding_side = true;
    const double right_s = getTraversedDistance(
      enable_avoidance, ref_point, right_angle, initial_dist, maps, search_expanding_side);
    const double left_s = getTraversedDistance(
      enable_avoidance, ref_point, left_angle, initial_dist, maps, search_expanding_side);
    if (left_s < right_s) {
      // Pick left side:
      right_bound = left_s;
      left_bound = getTraversedDistance(enable_avoidance, ref_point, left_angle, left_s, maps);
    } else {
      // Pick right side:
      left_bound = -right_s;
      right_bound = -getTraversedDistance(enable_avoidance, ref_point, right_angle, right_s, maps);
    }
  }
  return {left_bound, right_bound};
}

double MPTOptimizer::getClearance(
  const cv::Mat & clearance_map, const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & map_info, const double default_dist) const
{
  const auto image_point = util::transformMapToOptionalImage(map_point, map_info);
  if (!image_point) {
    return default_dist;
  }
  const float clearance =
    clearance_map.ptr<float>((int)image_point.get().y)[(int)image_point.get().x] *
    map_info.resolution;
  return clearance;
}

ObjectiveMatrix MPTOptimizer::getObjectiveMatrix(
  const Eigen::VectorXd & x0, const MPTMatrix & m) const
{
  const int DIM_U_N = m.Urefex.rows();
  const Eigen::MatrixXd CB = m.Cex * m.Bex;
  const Eigen::MatrixXd QCB = m.Qex * CB;
  // Eigen::MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex; // This calculation is heavy. looking for a good way.
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  Eigen::VectorXd f =
    (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Urefex.transpose() * m.R1ex;
  addSteerWeightF(&f);

  constexpr int num_lat_constraint = 3;
  const int num_objective_variables = DIM_U_N * (1 + num_lat_constraint);
  Eigen::MatrixXd extend_h = Eigen::MatrixXd::Zero(DIM_U_N, DIM_U_N);
  Eigen::VectorXd extend_f = Eigen::VectorXd::Ones(DIM_U_N);
  Eigen::MatrixXd concat_h =
    Eigen::MatrixXd::Zero(num_objective_variables, num_objective_variables);
  Eigen::VectorXd concat_f = Eigen::VectorXd::Zero(num_objective_variables);
  concat_h.block(0, 0, DIM_U_N, DIM_U_N) = H;
  concat_h.block(DIM_U_N, DIM_U_N, DIM_U_N, DIM_U_N) = extend_h;
  concat_h.block(DIM_U_N * 2, DIM_U_N * 2, DIM_U_N, DIM_U_N) = extend_h;
  concat_h.block(DIM_U_N * 3, DIM_U_N * 3, DIM_U_N, DIM_U_N) = extend_h;
  concat_f << f, mpt_param_ptr_->base_point_weight * extend_f,
    mpt_param_ptr_->top_point_weight * extend_f, mpt_param_ptr_->mid_point_weight * extend_f;
  ObjectiveMatrix obj_matrix;
  obj_matrix.hessian = concat_h;
  obj_matrix.gradient = {concat_f.data(), concat_f.data() + concat_f.rows()};

  return obj_matrix;
}

// Set constraint: lb <= Ax <= ub
// decision variable x := [u0, ..., uN-1 | z00, ..., z0N-1 | z10, ..., z1N-1 | z20, ..., z2N-1] \in \mathbb{R}^{N * (N_point + 1)}
ConstraintMatrix MPTOptimizer::getConstraintMatrix(
  const bool enable_avoidance, const Eigen::VectorXd & x0, const MPTMatrix & m, const CVMaps & maps,
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<autoware_planning_msgs::msg::PathPoint> & path_points,
  DebugData * debug_data) const
{
  std::vector<double> dist_vec{mpt_param_ptr_->base_point_dist_from_base_link,
    mpt_param_ptr_->top_point_dist_from_base_link,
    mpt_param_ptr_->mid_point_dist_from_base_link};

  const size_t N_ref = m.Urefex.rows();
  const size_t N_state = vehicle_model_ptr_->getDimX();
  const size_t N_point = dist_vec.size();
  const size_t N_dec = N_ref * (N_point + 1);

  const auto bounds = getReferenceBounds(enable_avoidance, ref_points, maps, debug_data);

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * N_ref * N_point + N_ref, N_dec);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(3 * N_ref * N_point + N_ref, -osqp::INF);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(3 * N_ref * N_point + N_ref, osqp::INF);

  // Define constraint matrices and vectors
  // Gap from reference point around vehicle base_link
  {
    // C := [I | O | O]
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      C(i, N_state * i) = 1;
    }
    // bias := Cast * (Aex * x0 + Wex)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);
    // A_blk := [C * Bex | I | O | O
    //          -C * Bex | I | O | O
    //               O   | I | O | O]
    Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(3 * N_ref, N_dec);
    A_blk.block(0, 0, N_ref, N_ref) = C * m.Bex;
    A_blk.block(0, N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(N_ref, 0, N_ref, N_ref) = -C * m.Bex;
    A_blk.block(N_ref, N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(2 * N_ref, N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    // lb_blk := [-bias + bounds.lb
    //             bias - bounds.ub
    //             0]
    Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(3 * N_ref);
    for (size_t i = 0; i < N_ref; ++i) {
      lb_blk(i) = -bias(i) + bounds[i].c0.lb;
      lb_blk(N_ref + i) = bias(i) - bounds[i].c0.ub;
    }
    // Assign
    A.block(0, 0, 3 * N_ref, N_dec) = A_blk;
    lb.segment(0, 3 * N_ref) = lb_blk;
  }

  // Gap from reference point around vehicle top
  {
    // C := diag([cos(alpha1) | l1*cos(alpha1) | 0])
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      Eigen::MatrixXd Cast = Eigen::MatrixXd::Zero(1, N_state);
      Cast(0, 0) = std::cos(ref_points[i].delta_yaw_from_p1);
      Cast(0, 1) = dist_vec[1] * std::cos(ref_points[i].delta_yaw_from_p1);
      C.block(i, N_state * i, 1, N_state) = Cast;
    }
    // bias := Cast * (Aex * x0 + Wex) - l1 * sin(alpha1)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);
    for (size_t i = 0; i < N_ref; ++i) {
      bias(i) -= dist_vec[1] * std::sin(ref_points[i].delta_yaw_from_p1);
    }
    // A_blk := [C * Bex | O | I | O
    //          -C * Bex | O | I | O
    //               O   | O | I | O]
    Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(3 * N_ref, N_dec);
    A_blk.block(0, 0, N_ref, N_ref) = C * m.Bex;
    A_blk.block(0, 2 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(N_ref, 0, N_ref, N_ref) = -C * m.Bex;
    A_blk.block(N_ref, 2 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(2 * N_ref, 2 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    // lb_blk := [-bias + bounds.lb
    //             bias - bounds.ub
    //             0]
    Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(3 * N_ref);
    for (size_t i = 0; i < N_ref; ++i) {
      lb_blk(i) = -bias(i) + bounds[i].c1.lb;
      lb_blk(N_ref + i) = bias(i) - bounds[i].c1.ub;
    }
    // Assign
    A.block(3 * N_ref, 0, 3 * N_ref, N_dec) = A_blk;
    lb.segment(3 * N_ref, 3 * N_ref) = lb_blk;
  }
  // Gap from reference point around vehicle middle
  {
    // C := [diag(cos(alpha2)) | diag(l2*cos(alpha2)) | O]
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      Eigen::MatrixXd Cast = Eigen::MatrixXd::Zero(1, N_state);
      Cast(0, 0) = std::cos(ref_points[i].delta_yaw_from_p2);
      Cast(0, 1) = dist_vec[2] * std::cos(ref_points[i].delta_yaw_from_p2);
      C.block(i, N_state * i, 1, N_state) = Cast;
    }
    // bias := Cast * (Aex * x0 + Wex) - l2 * sin(alpha2)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);
    for (size_t i = 0; i < N_ref; ++i) {
      bias(i) -= dist_vec[2] * std::sin(ref_points[i].delta_yaw_from_p2);
    }
    // A_blk := [C * Bex | O | O | I
    //          -C * Bex | O | O | I
    //               O   | O | O | I]
    Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(3 * N_ref, N_dec);
    A_blk.block(0, 0, N_ref, N_ref) = C * m.Bex;
    A_blk.block(0, 3 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(N_ref, 0, N_ref, N_ref) = -C * m.Bex;
    A_blk.block(N_ref, 3 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(2 * N_ref, 3 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    // lb_blk := [-bias + bounds.lb
    //             bias - bounds.ub
    //             0]
    Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(3 * N_ref);
    for (size_t i = 0; i < N_ref; ++i) {
      lb_blk(i) = -bias(i) + bounds[i].c2.lb;
      lb_blk(N_ref + i) = bias(i) - bounds[i].c2.ub;
    }
    // Assign
    A.block(2 * 3 * N_ref, 0, 3 * N_ref, N_dec) = A_blk;
    lb.segment(2 * 3 * N_ref, 3 * N_ref) = lb_blk;
  }

  // Fixed points constraint
  {
    // C := [I | O | O]
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      C(i, N_state * i) = 1;
    }
    // bias := Cast * (Aex * x0 + Wex)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);

    // Assign
    A.block(3 * N_point * N_ref, 0, N_ref, N_ref) = C * m.Bex;
    for (size_t i = 0; i < ref_points.size(); ++i) {
      if (ref_points[i].is_fix) {
        lb(3 * N_point * N_ref + i) = ref_points[i].fixing_lat - bias(i);
        ub(3 * N_point * N_ref + i) = ref_points[i].fixing_lat - bias(i);
      } else if (i == ref_points.size() - 1 && mpt_param_ptr_->is_hard_fix_terminal_point) {
        lb(3 * N_point * N_ref + i) = -bias(i);
        ub(3 * N_point * N_ref + i) = -bias(i);
      }
    }
  }

  ConstraintMatrix constraint_matrix;

  constraint_matrix.linear = A;

  for (size_t i = 0; i < lb.size(); ++i) {
    constraint_matrix.lower_bound.push_back(lb(i));
    constraint_matrix.upper_bound.push_back(ub(i));
  }

  return constraint_matrix;
}

std::vector<ReferencePoint> MPTOptimizer::getBaseReferencePoints(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points) const
{
  std::vector<ReferencePoint> reference_points;
  for (int i = 0; i < interpolated_points.size(); i++) {
    ReferencePoint ref_point;
    ref_point.p = interpolated_points[i];

    if (i > 0) {
      ref_point.q =
        util::getQuaternionFromPoints(interpolated_points[i], interpolated_points[i - 1]);
    } else if (i == 0 && interpolated_points.size() > 1) {
      ref_point.q =
        util::getQuaternionFromPoints(interpolated_points[i + 1], interpolated_points[i]);
    }
    ref_point.yaw = tf2::getYaw(ref_point.q);

    ref_point.v = points[util::getNearestIdx(points, interpolated_points[i])].twist.linear.x;
    reference_points.push_back(ref_point);
  }
  return reference_points;
}

double MPTOptimizer::getTraversedDistance(
  const bool enable_avoidance, const ReferencePoint & ref_point, const double traverse_angle,
  const double initial_value, const CVMaps & maps, const bool search_expanding_side) const
{
  constexpr double ds = 0.01;
  constexpr double lane_width = 10;
  auto n = static_cast<size_t>(lane_width / ds);

  double traversed_dist = initial_value;
  for (size_t i = 0; i < n; ++i) {
    traversed_dist += ds;
    geometry_msgs::msg::Point new_position;
    new_position.x = ref_point.p.x + traversed_dist * std::cos(traverse_angle);
    new_position.y = ref_point.p.y + traversed_dist * std::sin(traverse_angle);
    const double clearance = getClearance(maps.clearance_map, new_position, maps.map_info);
    double object_clearance =
      getClearance(maps.only_objects_clearance_map, new_position, maps.map_info);
    if (!enable_avoidance) {
      object_clearance = std::numeric_limits<double>::max();
    }
    if (search_expanding_side) {
      if (
        clearance > mpt_param_ptr_->clearance_from_road &&
        object_clearance > mpt_param_ptr_->clearance_from_object)
      {
        traversed_dist += ds;
        break;
      }
    } else {
      if (
        clearance < mpt_param_ptr_->clearance_from_road ||
        object_clearance < mpt_param_ptr_->clearance_from_object)
      {
        break;
      }
    }
  }
  return traversed_dist;
}
