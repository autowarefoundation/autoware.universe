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

#include "collision_free_path_planner/node.hpp"

#include "collision_free_path_planner/debug_marker.hpp"
#include "collision_free_path_planner/utils/geometry_utils.hpp"
#include "collision_free_path_planner/utils/trajectory_utils.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "rclcpp/time.hpp"

#include <chrono>
#include <limits>

namespace collision_free_path_planner
{
// TODO(murooka) check if velocity is updated while optimization is skipped.
// TODO(murooka) check if z is updated.
namespace
{
template <class T>
std::vector<T> concatVectors(const std::vector<T> & prev_vector, const std::vector<T> & next_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), prev_vector.begin(), prev_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), next_vector.begin(), next_vector.end());
  return concatenated_vector;
}

StringStamped createStringStamped(const rclcpp::Time & now, const std::string & data)
{
  StringStamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}
}  // namespace

CollisionFreePathPlanner::CollisionFreePathPlanner(const rclcpp::NodeOptions & node_options)
: Node("collision_free_path_planner", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  debug_data_ptr_(std::make_shared<DebugData>()),
  time_keeper_ptr_(std::make_shared<TimeKeeper>())
{
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);
  virtual_wall_pub_ = create_publisher<MarkerArray>("~/virtual_wall", 1);

  // interface subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", 1, std::bind(&CollisionFreePathPlanner::onPath, this, std::placeholders::_1));
  odom_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    [this](const Odometry::SharedPtr msg) { ego_state_ptr_ = msg; });

  // debug publisher
  debug_extended_traj_pub_ = create_publisher<Trajectory>("~/debug/extended_traj", 1);
  debug_markers_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_calculation_time_pub_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);

  {  // parameters
    // parameter for option
    enable_outside_drivable_area_stop_ =
      declare_parameter<bool>("option.enable_outside_drivable_area_stop");
    enable_smoothing_ = declare_parameter<bool>("option.enable_smoothing");
    enable_skip_optimization_ = declare_parameter<bool>("option.enable_skip_optimization");
    enable_reset_prev_optimization_ =
      declare_parameter<bool>("option.enable_reset_prev_optimization");

    // parameter for debug marker
    enable_pub_debug_marker_ = declare_parameter<bool>("option.debug.enable_pub_debug_marker");

    // parameter for debug info
    enable_debug_info_ = declare_parameter<bool>("option.debug.enable_debug_info");
    time_keeper_ptr_->enable_calculation_time_info =
      declare_parameter<bool>("option.debug.enable_calculation_time_info");

    // ego nearest search parameters declaration
    ego_nearest_param_ = EgoNearestParam(this);

    // trajectory parameters declaration
    traj_param_ = TrajectoryParam(this, vehicle_info_.vehicle_width_m);
  }

  // create core algorithm pointers with parameter declaration
  replan_checker_ptr_ = std::make_shared<ReplanChecker>(this, ego_nearest_param_);
  eb_path_smoother_ptr_ = std::make_shared<EBPathSmoother>(
    this, enable_debug_info_, ego_nearest_param_, traj_param_, time_keeper_ptr_);
  mpt_optimizer_ptr_ = std::make_shared<MPTOptimizer>(
    this, enable_debug_info_, ego_nearest_param_, vehicle_info_, traj_param_, debug_data_ptr_,
    time_keeper_ptr_);

  // first, reset planners
  // NOTE: This function must be called after core algorithms (e.g. mpt_optimizer_) have been
  // initialized.
  initializePlanning();

  // set parameter callback
  // NOTE: This function must be called after core algorithms (e.g. mpt_optimizer_) have been
  // initialized.
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&CollisionFreePathPlanner::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult CollisionFreePathPlanner::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // parameters
    // option parameter
    updateParam<bool>(
      parameters, "option.enable_outside_drivable_area_stop", enable_outside_drivable_area_stop_);
    updateParam<bool>(parameters, "option.enable_smoothing", enable_smoothing_);
    updateParam<bool>(parameters, "option.enable_skip_optimization", enable_skip_optimization_);
    updateParam<bool>(
      parameters, "option.enable_reset_prev_optimization", enable_reset_prev_optimization_);

    // debug marker
    updateParam<bool>(parameters, "option.debug.enable_pub_debug_marker", enable_pub_debug_marker_);

    // debug info
    updateParam<bool>(parameters, "option.debug.enable_debug_info", enable_debug_info_);
    updateParam<bool>(
      parameters, "option.debug.enable_calculation_time_info",
      time_keeper_ptr_->enable_calculation_time_info);
  }

  ego_nearest_param_.onParam(parameters);
  traj_param_.onParam(parameters);

  {  // core algorithms parameters
    replan_checker_ptr_->onParam(parameters);
    eb_path_smoother_ptr_->onParam(parameters);
    mpt_optimizer_ptr_->onParam(parameters);
  }

  // reset planners
  initializePlanning();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void CollisionFreePathPlanner::initializePlanning()
{
  RCLCPP_INFO(get_logger(), "Initialize planning");

  eb_path_smoother_ptr_->initialize(enable_debug_info_, traj_param_);
  mpt_optimizer_ptr_->initialize(enable_debug_info_, traj_param_);

  resetPrevData();
}

void CollisionFreePathPlanner::resetPrevData()
{
  eb_path_smoother_ptr_->resetPrevData();
  mpt_optimizer_ptr_->resetPrevData();

  prev_optimized_traj_points_ptr_ = nullptr;
}

void CollisionFreePathPlanner::onPath(const Path::SharedPtr path_ptr)
{
  time_keeper_ptr_->tic(__func__);

  // check if data is ready and valid
  if (!isDataReady(*path_ptr, *get_clock())) {
    return;
  }

  // 0. return if backward path
  // TODO(murooka): support backward path
  const auto is_driving_forward = driving_direction_checker_.isDrivingForward(path_ptr->points);
  if (!is_driving_forward) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Backward path is NOT supported. Just converting path to trajectory");

    const auto traj_points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);
    const auto output_traj_msg = trajectory_utils::createTrajectory(path_ptr->header, traj_points);
    traj_pub_->publish(output_traj_msg);
    return;
  }

  // 1. create planner data
  const auto planner_data = createPlannerData(*path_ptr);

  // 2. generate optimized trajectory
  const auto optimized_traj_points = generateOptimizedTrajectory(planner_data);

  // 3. extend trajectory to connect the optimized trajectory and the following path smoothly
  auto extended_traj_points = extendTrajectory(planner_data.traj_points, optimized_traj_points);

  // 4. set zero velocity after stop point
  // setZeroVelocityAfterStopPoint(extended_traj_points);

  // 5. publish debug data
  publishDebugData(planner_data.header);

  time_keeper_ptr_->toc(__func__, "");
  *time_keeper_ptr_ << "========================================";
  time_keeper_ptr_->endLine();

  // publish calculation_time
  // NOTE: This function must be called after measuring onPath calculation time
  const auto calculation_time_msg =
    createStringStamped(now(), time_keeper_ptr_->getAccumulatedTimeString());
  debug_calculation_time_pub_->publish(calculation_time_msg);

  const auto output_traj_msg =
    trajectory_utils::createTrajectory(path_ptr->header, extended_traj_points);
  traj_pub_->publish(output_traj_msg);
}

bool CollisionFreePathPlanner::isDataReady(const Path & path, rclcpp::Clock clock) const
{
  if (!ego_state_ptr_) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Waiting for ego pose and twist.");
    return false;
  }

  if (path.points.size() < 2) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Path points size is less than 1.");
    return false;
  }

  if (path.left_bound.empty() || path.right_bound.empty()) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), clock, 5000, "Left or right bound in path is empty.");
    return false;
  }

  return true;
}

PlannerData CollisionFreePathPlanner::createPlannerData(const Path & path) const
{
  // create planner data
  PlannerData planner_data;
  planner_data.header = path.header;
  planner_data.traj_points = trajectory_utils::convertToTrajectoryPoints(path.points);
  planner_data.left_bound = path.left_bound;
  planner_data.right_bound = path.right_bound;
  planner_data.ego_pose = ego_state_ptr_->pose.pose;
  planner_data.ego_vel = ego_state_ptr_->twist.twist.linear.x;

  debug_data_ptr_->ego_pose = planner_data.ego_pose;
  return planner_data;
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::generateOptimizedTrajectory(
  const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);

  const auto & traj_points = planner_data.traj_points;

  // 1. calculate trajectory with EB and MPT
  //    NOTE: This function may return previously optimized trajectory points.
  auto optimized_traj_points = optimizeTrajectory(planner_data);

  // TODO(murooka) plan always so that this implementation can be removed
  // 2. update velocity
  //    Even if optimization is skipped, velocity in trajectory points must be updated
  //    since velocity in input trajectory (path) may change
  // TODO(murooka)
  // applyInputVelocity(optimized_traj_points, traj_points);

  // 3. insert zero velocity when trajectory is over drivable area
  insertZeroVelocityOutsideDrivableArea(planner_data, optimized_traj_points);

  // 4. publish debug marker
  publishDebugMarkerInOptimization(planner_data, optimized_traj_points);

  time_keeper_ptr_->toc(__func__, " ");
  return optimized_traj_points;
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::optimizeTrajectory(
  const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);
  const auto & p = planner_data;

  // 1. check if replan (= optimizatoin) is required
  const bool reset_prev_optimization = replan_checker_ptr_->isResetRequired(planner_data);
  if (enable_reset_prev_optimization_ || reset_prev_optimization) {
    // NOTE: always replan when resetting previous optimization
    resetPrevData();
  } else {
    // check replan when not resetting previous optimization
    const bool is_replan_required =
      replan_checker_ptr_->isReplanRequired(planner_data, now(), prev_optimized_traj_points_ptr_);
    if (!is_replan_required) {
      return getPrevOptimizedTrajectory(p.traj_points);
    }
  }

  if (enable_skip_optimization_) {
    return p.traj_points;
  }

  // 2. smooth trajectory with elastic band
  const auto eb_traj =
    enable_smoothing_ ? eb_path_smoother_ptr_->getEBTrajectory(planner_data) : p.traj_points;
  if (!eb_traj) {
    return getPrevOptimizedTrajectory(p.traj_points);
  }

  // 3. make trajectory kinematically-feasible and collision-free (= inside the drivable area)
  //    with model predictive trajectory
  const auto mpt_traj = mpt_optimizer_ptr_->getModelPredictiveTrajectory(planner_data, *eb_traj);
  if (!mpt_traj) {
    return getPrevOptimizedTrajectory(p.traj_points);
  }

  // 4. make prev trajectories
  prev_optimized_traj_points_ptr_ = std::make_shared<std::vector<TrajectoryPoint>>(*mpt_traj);

  time_keeper_ptr_->toc(__func__, "    ");
  return *mpt_traj;
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::getPrevOptimizedTrajectory(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (prev_optimized_traj_points_ptr_) {
    return *prev_optimized_traj_points_ptr_;
  }

  // TODO(murooka) no need to generate post procesed trajectory?
  return traj_points;
}

/*
void CollisionFreePathPlanner::applyInputVelocity(
  std::vector<TrajectoryPoint> & traj_points, const std::vector<TrajectoryPoint> & traj_points)
const
{
  time_keeper_ptr_->tic(__func__);

  for (size_t i = 0; i < traj_points.size(); i++) {
    const size_t nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, traj_points.at(i).pose, traj_param_.delta_dist_threshold_for_closest_point,
      traj_param_.delta_yaw_threshold_for_closest_point);

    // TODO(murooka)
    // add this line not to exceed max index size
    const size_t max_idx = std::min(nearest_seg_idx + 1, traj_points.size() - 1);
    // NOTE: std::max, not std::min, is used here since traj_points' sampling width may be longer
    // than path_points' sampling width. A zero velocity point is guaranteed to be inserted in an
    // output trajectory in the alignVelocity function
    traj_points.at(i).longitudinal_velocity_mps = std::max(
      traj_points.at(nearest_seg_idx).longitudinal_velocity_mps,
      traj_points.at(max_idx).longitudinal_velocity_mps);

    // TODO(murooka) insert stop point explicitly
  }

  time_keeper_ptr_->toc(__func__, "    ");
}
*/

void CollisionFreePathPlanner::insertZeroVelocityOutsideDrivableArea(
  const PlannerData & planner_data, std::vector<TrajectoryPoint> & optimized_traj_points)
{
  time_keeper_ptr_->tic(__func__);

  if (!enable_outside_drivable_area_stop_) {
    return;
  }

  if (optimized_traj_points.empty()) {
    return;
  }

  // 1. calculate ego_index nearest to optimized_traj_points
  const size_t ego_idx = trajectory_utils::findEgoIndex(
    optimized_traj_points, planner_data.ego_pose, ego_nearest_param_);

  // 2. calculate an end point to check being outside the drivable are
  // NOTE: Some end trajectory points should be ignored to check if being outside the drivable area
  //       since these points tend to be outside drivable area when end reference points have high
  //       curvature.
  const int end_idx = std::max(10, static_cast<int>(optimized_traj_points.size()) - 5);

  // 3. assign zero velocity to the first point being outside the drivable area
  const auto first_outside_idx = [&]() -> std::optional<size_t> {
    for (size_t i = ego_idx; i < static_cast<size_t>(end_idx); ++i) {
      auto & traj_point = optimized_traj_points.at(i);

      // check if the footprint is outside the drivable area
      const bool is_outside = geometry_utils::isOutsideDrivableAreaFromRectangleFootprint(
        traj_point.pose, planner_data.left_bound, planner_data.right_bound, vehicle_info_);

      if (is_outside) {
        publishVirtualWall(traj_point.pose);
        return i;
      }
    }
    return std::nullopt;
  }();

  if (first_outside_idx) {
    for (size_t i = *first_outside_idx; i < optimized_traj_points.size(); ++i) {
      optimized_traj_points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }

  time_keeper_ptr_->toc(__func__, "    ");
}

void CollisionFreePathPlanner::publishVirtualWall(const geometry_msgs::msg::Pose & stop_pose) const
{
  time_keeper_ptr_->tic(__func__);

  const auto virtual_wall_marker = motion_utils::createStopVirtualWallMarker(
    stop_pose, "outside drivable area", now(), 0, vehicle_info_.max_longitudinal_offset_m);

  virtual_wall_pub_->publish(virtual_wall_marker);
  time_keeper_ptr_->toc(__func__, "      ");
}

void CollisionFreePathPlanner::publishDebugMarkerInOptimization(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points)
{
  if (!enable_pub_debug_marker_) {
    return;
  }

  time_keeper_ptr_->tic(__func__);
  const auto & p = planner_data;

  // debug marker
  time_keeper_ptr_->tic("getDebugMarker");
  const auto & debug_marker = getDebugMarker(*debug_data_ptr_, traj_points, vehicle_info_);
  time_keeper_ptr_->toc("getDebugMarker", "      ");

  time_keeper_ptr_->tic("publishDebugMarker");
  debug_markers_pub_->publish(debug_marker);
  time_keeper_ptr_->toc("publishDebugMarker", "      ");

  time_keeper_ptr_->toc(__func__, "    ");
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::extendTrajectory(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points)
{
  time_keeper_ptr_->tic(__func__);

  const auto & joint_start_pose = optimized_traj_points.back().pose;

  // calculate end idx of optimized points on path points
  const auto opt_end_traj_seg_idx = motion_utils::findNearestSegmentIndex(
    traj_points, joint_start_pose, traj_param_.delta_dist_threshold_for_closest_point,
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (!opt_end_traj_seg_idx) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "Not extend trajectory since could not find nearest idx from last opt point");
    return std::vector<TrajectoryPoint>{};
  }
  const size_t end_traj_seg_idx = opt_end_traj_seg_idx.get();

  // crop forward trajectory
  const auto forward_traj_points = [&]() -> std::vector<TrajectoryPoint> {
    constexpr double joint_traj_length_for_resampling = 10.0;
    const auto forward_traj_points = trajectory_utils::cropPointsAfterOffsetPoint(
      traj_points, joint_start_pose.position, end_traj_seg_idx, joint_traj_length_for_resampling);

    if (forward_traj_points.empty()) {  // compensate goal pose
      // TODO(murooka) optimization last point may be traj last point
      std::vector<TrajectoryPoint>{traj_points.back()};
    }

    return forward_traj_points;
  }();

  const auto extended_traj_points = concatVectors(optimized_traj_points, forward_traj_points);

  const auto resampled_traj_points = trajectory_utils::resampleTrajectoryPoints(
    extended_traj_points, traj_param_.output_delta_arc_length);

  // TODO(murooka) update velocity on joint

  debug_data_ptr_->extended_traj_points = forward_traj_points;
  time_keeper_ptr_->toc(__func__, "  ");
  return resampled_traj_points;
}

void CollisionFreePathPlanner::publishDebugData(const Header & header) const
{
  time_keeper_ptr_->tic(__func__);

  // publish trajectories
  const auto debug_extended_traj =
    trajectory_utils::createTrajectory(header, debug_data_ptr_->extended_traj_points);
  debug_extended_traj_pub_->publish(debug_extended_traj);

  time_keeper_ptr_->toc(__func__, "  ");
}
}  // namespace collision_free_path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(collision_free_path_planner::CollisionFreePathPlanner)
