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

#include "collision_free_path_planner/node.hpp"

#include "collision_free_path_planner/utils/cv_utils.hpp"
#include "collision_free_path_planner/utils/debug_utils.hpp"
#include "collision_free_path_planner/utils/utils.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/motion_utils.hpp"
#include "rclcpp/time.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/ros/update_param.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace collision_free_path_planner
{
// TODO(murooka) check if velocity is updated while optimization is skipped.
namespace
{
[[maybe_unused]] void fillYawInTrajectoryPoint(std::vector<TrajectoryPoint> & traj_points)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & traj_point : traj_points) {
    points.push_back(traj_point.pose.position);
  }
  const auto yaw_vec = interpolation::splineYawFromPoints(points);

  for (size_t i = 0; i < traj_points.size(); ++i) {
    traj_points.at(i).pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw_vec.at(i));
  }
}

template <class T>
[[maybe_unused]] size_t findNearestIndexWithSoftYawConstraints(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx_optional =
    motion_utils::findNearestIndex(points, pose, dist_threshold, yaw_threshold);
  return nearest_idx_optional ? *nearest_idx_optional
                              : motion_utils::findNearestIndex(points, pose.position);
}

template <>
[[maybe_unused]] size_t findNearestIndexWithSoftYawConstraints(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold, const double yaw_threshold)
{
  const auto points_with_yaw = points_utils::convertToPosesWithYawEstimation(points);

  return findNearestIndexWithSoftYawConstraints(
    points_with_yaw, pose, dist_threshold, yaw_threshold);
}

template <class T>
[[maybe_unused]] size_t findNearestSegmentIndexWithSoftYawConstraints(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx_optional =
    motion_utils::findNearestSegmentIndex(points, pose, dist_threshold, yaw_threshold);
  return nearest_idx_optional ? *nearest_idx_optional
                              : motion_utils::findNearestSegmentIndex(points, pose.position);
}

template <>
[[maybe_unused]] size_t findNearestSegmentIndexWithSoftYawConstraints(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold, const double yaw_threshold)
{
  const auto points_with_yaw = points_utils::convertToPosesWithYawEstimation(points);

  return findNearestSegmentIndexWithSoftYawConstraints(
    points_with_yaw, pose, dist_threshold, yaw_threshold);
}

Trajectory createTrajectory(
  const std_msgs::msg::Header & header, const std::vector<TrajectoryPoint> & traj_points)
{
  auto traj = motion_utils::convertToTrajectory(traj_points);
  traj.header = header;

  return traj;
}

[[maybe_unused]] std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = motion_utils::resampleTrajectory(traj, interval);

  // convert Trajectory to std::vector<TrajectoryPoint>
  std::vector<TrajectoryPoint> resampled_traj_points;
  for (const auto & point : resampled_traj.points) {
    resampled_traj_points.push_back(point);
  }

  return resampled_traj_points;
}

std::vector<TrajectoryPoint> concatTrajectory(
  const std::vector<TrajectoryPoint> & prev_traj_points,
  const std::vector<TrajectoryPoint> & next_traj_points)
{
  std::vector<TrajectoryPoint> concatenated_traj_points;
  concatenated_traj_points.insert(
    concatenated_traj_points.end(), prev_traj_points.begin(), prev_traj_points.end());
  concatenated_traj_points.insert(
    concatenated_traj_points.end(), next_traj_points.begin(), next_traj_points.end());
  return concatenated_traj_points;
}
}  // namespace

CollisionFreePathPlanner::CollisionFreePathPlanner(const rclcpp::NodeOptions & node_options)
: Node("collision_free_path_planner", node_options),
  logger_ros_clock_(RCL_ROS_TIME),
  eb_solved_count_(0)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // qos
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  // publisher to other nodes
  traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);

  // debug publisher
  debug_eb_traj_pub_ = create_publisher<Trajectory>("~/debug/eb_trajectory", durable_qos);
  debug_extended_fixed_traj_pub_ = create_publisher<Trajectory>("~/debug/extended_fixed_traj", 1);
  debug_extended_non_fixed_traj_pub_ =
    create_publisher<Trajectory>("~/debug/extended_non_fixed_traj", 1);
  debug_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", durable_qos);
  debug_wall_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/wall_marker", durable_qos);
  debug_clearance_map_pub_ = create_publisher<OccupancyGrid>("~/debug/clearance_map", durable_qos);
  debug_object_clearance_map_pub_ =
    create_publisher<OccupancyGrid>("~/debug/object_clearance_map", durable_qos);
  debug_area_with_objects_pub_ =
    create_publisher<OccupancyGrid>("~/debug/area_with_objects", durable_qos);
  debug_msg_pub_ =
    create_publisher<tier4_debug_msgs::msg::StringStamped>("~/debug/calculation_time", 1);

  // subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&CollisionFreePathPlanner::onPath, this, std::placeholders::_1));
  odom_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&CollisionFreePathPlanner::onOdometry, this, std::placeholders::_1));
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&CollisionFreePathPlanner::onObjects, this, std::placeholders::_1));

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  {  // vehicle param
    vehicle_param_ = VehicleParam{};
    vehicle_param_.width = vehicle_info.vehicle_width_m;
    vehicle_param_.length = vehicle_info.vehicle_length_m;
    vehicle_param_.wheelbase = vehicle_info.wheel_base_m;
    vehicle_param_.rear_overhang = vehicle_info.rear_overhang_m;
    vehicle_param_.front_overhang = vehicle_info.front_overhang_m;
  }

  {  // option parameter
    enable_outside_drivable_area_stop_ =
      declare_parameter<bool>("option.enable_outside_drivable_area_stop");
    enable_avoidance_ = declare_parameter<bool>("option.enable_avoidance");
    enable_smoothing_ = declare_parameter<bool>("option.enable_smoothing");
    enable_skip_optimization_ = declare_parameter<bool>("option.enable_skip_optimization");
    enable_reset_prev_optimization_ =
      declare_parameter<bool>("option.enable_reset_prev_optimization");

    // debug marker
    enable_debug_marker_pub_ = declare_parameter<bool>("option.debug.enable_debug_marker_pub");
    enable_clearance_map_pub_ = declare_parameter<bool>("option.debug.enable_clearance_map_pub");
    enable_object_clearance_map_pub_ =
      declare_parameter<bool>("option.debug.enable_object_clearance_map_pub");
    enable_area_with_objects_pub_ =
      declare_parameter<bool>("option.debug.enable_area_with_objects_pub");

    // debug info
    enable_debug_info_ = declare_parameter<bool>("option.debug.enable_debug_info");
    enable_calculation_time_info_ =
      declare_parameter<bool>("option.debug.enable_calculation_time_info");
  }

  {  // ego nearest search param
    const double ego_nearest_dist_threshold =
      declare_parameter<double>("ego_nearest_dist_threshold");
    const double ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
    ego_nearest_param_ = EgoNearestParam{ego_nearest_dist_threshold, ego_nearest_yaw_threshold};
  }

  {  // trajectory parameter
    traj_param_ = TrajectoryParam{};

    // common
    traj_param_.num_sampling_points = declare_parameter<int>("common.num_sampling_points");
    traj_param_.output_traj_length = declare_parameter<double>("common.output_traj_length");
    traj_param_.forward_fixing_min_distance =
      declare_parameter<double>("common.forward_fixing_min_distance");
    traj_param_.forward_fixing_min_time =
      declare_parameter<double>("common.forward_fixing_min_time");
    traj_param_.output_backward_traj_length =
      declare_parameter<double>("common.output_backward_traj_length");
    traj_param_.output_delta_arc_length =
      declare_parameter<double>("common.output_delta_arc_length");

    traj_param_.delta_dist_threshold_for_closest_point =
      declare_parameter<double>("common.delta_dist_threshold_for_closest_point");
    traj_param_.delta_yaw_threshold_for_closest_point =
      declare_parameter<double>("common.delta_yaw_threshold_for_closest_point");
    traj_param_.delta_yaw_threshold_for_straight =
      declare_parameter<double>("common.delta_yaw_threshold_for_straight");

    // object
    traj_param_.max_avoiding_ego_velocity_ms =
      declare_parameter<double>("object.max_avoiding_ego_velocity_ms");
    traj_param_.max_avoiding_objects_velocity_ms =
      declare_parameter<double>("object.max_avoiding_objects_velocity_ms");
    traj_param_.is_avoiding_unknown =
      declare_parameter<bool>("object.avoiding_object_type.unknown", true);
    traj_param_.is_avoiding_car = declare_parameter<bool>("object.avoiding_object_type.car", true);
    traj_param_.is_avoiding_truck =
      declare_parameter<bool>("object.avoiding_object_type.truck", true);
    traj_param_.is_avoiding_bus = declare_parameter<bool>("object.avoiding_object_type.bus", true);
    traj_param_.is_avoiding_bicycle =
      declare_parameter<bool>("object.avoiding_object_type.bicycle", true);
    traj_param_.is_avoiding_motorbike =
      declare_parameter<bool>("object.avoiding_object_type.motorbike", true);
    traj_param_.is_avoiding_pedestrian =
      declare_parameter<bool>("object.avoiding_object_type.pedestrian", true);
    traj_param_.is_avoiding_animal =
      declare_parameter<bool>("object.avoiding_object_type.animal", true);

    // TODO(murooka) remove this after refactoring EB
    traj_param_.ego_nearest_dist_threshold = ego_nearest_param_.dist_threshold;
    traj_param_.ego_nearest_yaw_threshold = ego_nearest_param_.yaw_threshold;
  }

  {  // elastic band parameter
    eb_param_ = EBParam{};

    // common
    eb_param_.num_joint_buffer_points =
      declare_parameter<int>("advanced.eb.common.num_joint_buffer_points");
    eb_param_.num_offset_for_begin_idx =
      declare_parameter<int>("advanced.eb.common.num_offset_for_begin_idx");
    eb_param_.delta_arc_length_for_eb =
      declare_parameter<double>("advanced.eb.common.delta_arc_length_for_eb");
    eb_param_.num_sampling_points_for_eb =
      declare_parameter<int>("advanced.eb.common.num_sampling_points_for_eb");

    // clearance
    eb_param_.clearance_for_straight_line =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_straight_line");
    eb_param_.clearance_for_joint =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_joint");
    eb_param_.clearance_for_only_smoothing =
      declare_parameter<double>("advanced.eb.clearance.clearance_for_only_smoothing");

    // qp
    eb_param_.qp_param.max_iteration = declare_parameter<int>("advanced.eb.qp.max_iteration");
    eb_param_.qp_param.eps_abs = declare_parameter<double>("advanced.eb.qp.eps_abs");
    eb_param_.qp_param.eps_rel = declare_parameter<double>("advanced.eb.qp.eps_rel");

    // other
    eb_param_.clearance_for_fixing = 0.0;
  }

  // TODO(murooka) tune this param when avoiding with collision_free_path_planner
  traj_param_.center_line_width = vehicle_param_.width;

  mpt_optimizer_ptr_ = std::make_unique<MPTOptimizer>(
    this, enable_debug_info_, ego_nearest_param_, vehicle_info, traj_param_);
  resetPlanning();

  replan_checker_ = std::make_shared<ReplanChecker>(*this, ego_nearest_param_);

  // set parameter callback
  //! NOTE: This function must be called after algorithms (e.g. mpt_optimizer) are initialized.
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&CollisionFreePathPlanner::onParam, this, std::placeholders::_1));

  // self_pose_listener_.waitForFirstPose();
}

rcl_interfaces::msg::SetParametersResult CollisionFreePathPlanner::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option parameter
    updateParam<bool>(
      parameters, "option.enable_outside_drivable_area_stop", enable_outside_drivable_area_stop_);
    updateParam<bool>(parameters, "option.enable_avoidance", enable_avoidance_);
    updateParam<bool>(parameters, "option.enable_smoothing", enable_smoothing_);
    updateParam<bool>(parameters, "option.enable_skip_optimization", enable_skip_optimization_);
    updateParam<bool>(
      parameters, "option.enable_reset_prev_optimization", enable_reset_prev_optimization_);

    // debug marker
    updateParam<bool>(parameters, "option.debug.enable_debug_marker_pub", enable_debug_marker_pub_);
    updateParam<bool>(
      parameters, "option.debug.enable_clearance_map_pub", enable_clearance_map_pub_);
    updateParam<bool>(
      parameters, "option.debug.enable_object_clearance_map_pub", enable_object_clearance_map_pub_);
    updateParam<bool>(
      parameters, "option.debug.enable_area_with_objects_pub", enable_area_with_objects_pub_);

    // debug info
    updateParam<bool>(parameters, "option.debug.enable_debug_info", enable_debug_info_);
    updateParam<bool>(
      parameters, "option.debug.enable_calculation_time_info", enable_calculation_time_info_);
  }

  {  // trajectory parameter
    // common
    updateParam<int>(parameters, "common.num_sampling_points", traj_param_.num_sampling_points);
    updateParam<double>(parameters, "common.output_traj_length", traj_param_.output_traj_length);
    updateParam<double>(
      parameters, "common.forward_fixing_min_distance", traj_param_.forward_fixing_min_distance);
    updateParam<double>(
      parameters, "common.forward_fixing_min_time", traj_param_.forward_fixing_min_time);
    updateParam<double>(
      parameters, "common.output_backward_traj_length", traj_param_.output_backward_traj_length);
    updateParam<double>(
      parameters, "common.output_delta_arc_length", traj_param_.output_delta_arc_length);

    updateParam<double>(
      parameters, "common.delta_dist_threshold_for_closest_point",
      traj_param_.delta_dist_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_closest_point",
      traj_param_.delta_yaw_threshold_for_closest_point);
    updateParam<double>(
      parameters, "common.delta_yaw_threshold_for_straight",
      traj_param_.delta_yaw_threshold_for_straight);

    // object
    updateParam<double>(
      parameters, "object.max_avoiding_ego_velocity_ms", traj_param_.max_avoiding_ego_velocity_ms);
    updateParam<double>(
      parameters, "object.max_avoiding_objects_velocity_ms",
      traj_param_.max_avoiding_objects_velocity_ms);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.unknown", traj_param_.is_avoiding_unknown);
    updateParam<bool>(parameters, "object.avoiding_object_type.car", traj_param_.is_avoiding_car);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.truck", traj_param_.is_avoiding_truck);
    updateParam<bool>(parameters, "object.avoiding_object_type.bus", traj_param_.is_avoiding_bus);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.bicycle", traj_param_.is_avoiding_bicycle);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.motorbike", traj_param_.is_avoiding_motorbike);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.pedestrian", traj_param_.is_avoiding_pedestrian);
    updateParam<bool>(
      parameters, "object.avoiding_object_type.animal", traj_param_.is_avoiding_animal);
  }

  {  // elastic band parameter
    // common
    updateParam<int>(
      parameters, "advanced.eb.common.num_joint_buffer_points", eb_param_.num_joint_buffer_points);
    updateParam<int>(
      parameters, "advanced.eb.common.num_offset_for_begin_idx",
      eb_param_.num_offset_for_begin_idx);
    updateParam<double>(
      parameters, "advanced.eb.common.delta_arc_length_for_eb", eb_param_.delta_arc_length_for_eb);
    updateParam<int>(
      parameters, "advanced.eb.common.num_sampling_points_for_eb",
      eb_param_.num_sampling_points_for_eb);

    // clearance
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_straight_line",
      eb_param_.clearance_for_straight_line);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_joint", eb_param_.clearance_for_joint);
    updateParam<double>(
      parameters, "advanced.eb.clearance.clearance_for_only_smoothing",
      eb_param_.clearance_for_only_smoothing);

    // qp
    updateParam<int>(parameters, "advanced.eb.qp.max_iteration", eb_param_.qp_param.max_iteration);
    updateParam<double>(parameters, "advanced.eb.qp.eps_abs", eb_param_.qp_param.eps_abs);
    updateParam<double>(parameters, "advanced.eb.qp.eps_rel", eb_param_.qp_param.eps_rel);
  }

  // mpt
  mpt_optimizer_ptr_->onParam(parameters);

  // initialize planners
  resetPlanning();

  // update parameters for replan checker
  replan_checker_->onParam(parameters);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void CollisionFreePathPlanner::onOdometry(const Odometry::SharedPtr msg)
{
  current_twist_ptr_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

void CollisionFreePathPlanner::onObjects(const PredictedObjects::SharedPtr msg)
{
  objects_ptr_ = std::make_unique<PredictedObjects>(*msg);
}

void CollisionFreePathPlanner::resetPlanning()
{
  RCLCPP_WARN(get_logger(), "[CollisionFreePathPlanner] Reset planning");

  costmap_generator_ptr_ = std::make_unique<CostmapGenerator>();

  eb_path_optimizer_ptr_ = std::make_unique<EBPathOptimizer>(
    enable_debug_info_, ego_nearest_param_, traj_param_, eb_param_);

  mpt_optimizer_ptr_->reset(enable_debug_info_, traj_param_);

  resetPrevOptimization();
}

void CollisionFreePathPlanner::resetPrevOptimization()
{
  prev_eb_traj_ptr_ = nullptr;
  prev_mpt_trajs_ptr_ = nullptr;
  eb_solved_count_ = 0;
}

void CollisionFreePathPlanner::onPath(const Path::SharedPtr path_ptr)
{
  stop_watch_.tic(__func__);

  // check if data is ready
  if (!isDataReady()) {
    return;
  }

  // check if data is valid
  if (path_ptr->points.empty() || path_ptr->drivable_area.data.empty()) {
    return;
  }

  // create planner data
  PlannerData planner_data;
  planner_data.path = *path_ptr;
  planner_data.ego_pose = self_pose_listener_.getCurrentPose()->pose;
  planner_data.ego_vel = current_twist_ptr_->twist.linear.x;
  planner_data.objects = objects_ptr_->objects;
  planner_data.enable_avoidance = enable_avoidance_;

  // create debug data
  debug_data_ = DebugData();
  debug_data_.init(enable_calculation_time_info_, planner_data.ego_pose);

  // generate trajectory
  const auto output_traj_msg = generateTrajectory(planner_data);

  // publish debug data
  publishDebugDataInMain(*path_ptr);

  {  // print and publish debug msg
    debug_data_.msg_stream << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n"
                           << "========================================";
    tier4_debug_msgs::msg::StringStamped debug_msg_msg;
    debug_msg_msg.stamp = get_clock()->now();
    debug_msg_msg.data = debug_data_.msg_stream.getString();
    debug_msg_pub_->publish(debug_msg_msg);
  }

  traj_pub_->publish(output_traj_msg);
}

bool CollisionFreePathPlanner::isDataReady()
{
  const auto is_data_missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", name);
    return false;
  };

  if (!current_twist_ptr_) {
    return is_data_missing("self_twist");
  }

  if (!objects_ptr_) {
    return is_data_missing("dynamic_object");
  }

  const auto ego_pose = self_pose_listener_.getCurrentPose();
  if (!ego_pose) {
    return is_data_missing("self_pose");
  }

  return true;
}

Trajectory CollisionFreePathPlanner::generateTrajectory(const PlannerData & planner_data)
{
  const auto & p = planner_data;

  // TODO(someone): support backward path
  const auto is_driving_forward = motion_utils::isDrivingForward(p.path.points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.get() : is_driving_forward_;
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[CollisionFreePathPlanner] Backward path is NOT supported. Just converting path to "
      "trajectory");

    const auto traj_points = points_utils::convertToTrajectoryPoints(p.path.points);
    return createTrajectory(p.path.header, traj_points);
  }

  // generate optimized trajectory
  const auto optimized_traj_points = generateOptimizedTrajectory(planner_data);

  // generate post processed trajectory
  const auto post_processed_traj_points =
    generatePostProcessedTrajectory(planner_data, optimized_traj_points);

  // convert to output msg type
  return createTrajectory(p.path.header, post_processed_traj_points);
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::generateOptimizedTrajectory(
  const PlannerData & planner_data)
{
  stop_watch_.tic(__func__);

  const auto & path = planner_data.path;

  // check if optimization is required or not.
  // NOTE: previous trajectories information will be reset in some cases.
  const bool is_replan_required =
    replan_checker_->isReplanRequired(planner_data, now(), prev_mpt_trajs_ptr_);
  if (!is_replan_required) {
    return getPrevOptimizedTrajectory(path.points);
  }

  const bool reset_prev_optimization_required = replan_checker_->isResetOptimizationRequired();
  if (enable_reset_prev_optimization_ || reset_prev_optimization_required) {
    resetPrevOptimization();
  }

  // create clearance maps
  const CVMaps cv_maps = costmap_generator_ptr_->getMaps(
    enable_avoidance_, path, planner_data.objects, traj_param_, debug_data_);

  // calculate trajectory with EB and MPT
  auto mpt_traj = optimizeTrajectory(planner_data, cv_maps);

  // calculate velocity
  // NOTE: Velocity is not considered in optimization.
  calcVelocity(path.points, mpt_traj);

  // insert 0 velocity when trajectory is over drivable area
  if (enable_outside_drivable_area_stop_) {
    insertZeroVelocityOutsideDrivableArea(planner_data, mpt_traj, cv_maps);
  }

  publishDebugDataInOptimization(planner_data, mpt_traj);

  debug_data_.msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return mpt_traj;
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::optimizeTrajectory(
  const PlannerData & planner_data, const CVMaps & cv_maps)
{
  stop_watch_.tic(__func__);

  const auto & p = planner_data;

  if (enable_skip_optimization_) {
    return points_utils::convertToTrajectoryPoints(p.path.points);
  }

  // EB: smooth trajectory if enable_smoothing is true
  const auto eb_traj = [&]() -> boost::optional<std::vector<TrajectoryPoint>> {
    // TODO(murooka) enable EB
    /*
if (enable_smoothing_) {
return eb_path_optimizer_ptr_->getEBTrajectory(planner_data, prev_eb_traj_ptr_, debug_data_);
}
    */
    return points_utils::convertToTrajectoryPoints(p.path.points);
  }();
  if (!eb_traj) {
    return getPrevOptimizedTrajectory(p.path.points);
  }

  // EB has to be solved twice before solving MPT with fixed points
  // since the result of EB is likely to change with/without fixing (1st/2nd EB)
  // that makes MPT fixing points worse.
  if (eb_solved_count_ < 2) {
    eb_solved_count_++;

    if (prev_mpt_trajs_ptr_) {
      prev_mpt_trajs_ptr_->mpt.clear();
      prev_mpt_trajs_ptr_->ref_points.clear();
    }
  }

  // MPT: optimize trajectory to be kinematically feasible and collision free
  const auto mpt_trajs = mpt_optimizer_ptr_->getModelPredictiveTrajectory(
    planner_data, eb_traj.get(), prev_mpt_trajs_ptr_, cv_maps, debug_data_);
  if (!mpt_trajs) {
    return getPrevOptimizedTrajectory(p.path.points);
  }

  // make prev trajectories
  prev_eb_traj_ptr_ = std::make_shared<std::vector<TrajectoryPoint>>(eb_traj.get());
  prev_mpt_trajs_ptr_ = std::make_shared<MPTTrajs>(mpt_trajs.get());

  // debug data
  debug_data_.eb_traj = eb_traj.get();

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";

  return mpt_trajs->mpt;
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::getPrevOptimizedTrajectory(
  const std::vector<PathPoint> & path_points) const
{
  if (prev_mpt_trajs_ptr_) {
    return prev_mpt_trajs_ptr_->mpt;
  }
  return points_utils::convertToTrajectoryPoints(path_points);
}

void CollisionFreePathPlanner::calcVelocity(
  const std::vector<PathPoint> & path_points, std::vector<TrajectoryPoint> & traj_points) const
{
  for (size_t i = 0; i < traj_points.size(); i++) {
    const size_t nearest_seg_idx = findNearestSegmentIndexWithSoftYawConstraints(
      path_points, traj_points.at(i).pose, traj_param_.delta_dist_threshold_for_closest_point,
      traj_param_.delta_yaw_threshold_for_closest_point);

    // add this line not to exceed max index size
    const size_t max_idx = std::min(nearest_seg_idx + 1, path_points.size() - 1);
    // NOTE: std::max, not std::min, is used here since traj_points' sampling width may be longer
    // than path_points' sampling width. A zero velocity point is guaranteed to be inserted in an
    // output trajectory in the alignVelocity function
    traj_points.at(i).longitudinal_velocity_mps = std::max(
      path_points.at(nearest_seg_idx).longitudinal_velocity_mps,
      path_points.at(max_idx).longitudinal_velocity_mps);
  }
}

void CollisionFreePathPlanner::insertZeroVelocityOutsideDrivableArea(
  const PlannerData & planner_data, std::vector<TrajectoryPoint> & traj_points,
  const CVMaps & cv_maps)
{
  if (traj_points.empty()) {
    return;
  }

  stop_watch_.tic(__func__);

  const auto & map_info = cv_maps.map_info;
  const auto & road_clearance_map = cv_maps.clearance_map;

  const size_t nearest_idx =
    points_utils::findEgoIndex(traj_points, planner_data.ego_pose, ego_nearest_param_);

  // NOTE: Some end trajectory points will be ignored to check if outside the drivable area
  //       since these points might be outside drivable area if only end reference points have high
  //       curvature.
  const size_t end_idx = [&]() {
    const size_t enough_long_points_num =
      static_cast<size_t>(traj_param_.num_sampling_points * 0.7);
    constexpr size_t end_ignored_points_num = 5;
    if (traj_points.size() > enough_long_points_num) {
      return std::max(static_cast<size_t>(0), traj_points.size() - end_ignored_points_num);
    }
    return traj_points.size();
  }();

  for (size_t i = nearest_idx; i < end_idx; ++i) {
    const auto & traj_point = traj_points.at(i);

    // calculate the first point being outside drivable area
    const bool is_outside = cv_drivable_area_utils::isOutsideDrivableAreaFromRectangleFootprint(
      traj_point, road_clearance_map, map_info, vehicle_param_);

    // only insert zero velocity to the first point outside drivable area
    if (is_outside) {
      traj_points[i].longitudinal_velocity_mps = 0.0;
      debug_data_.stop_pose_by_drivable_area = traj_points[i].pose;
      break;
    }
  }

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
}

void CollisionFreePathPlanner::publishDebugDataInOptimization(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points)
{
  stop_watch_.tic(__func__);

  const auto & p = planner_data;

  {  // publish trajectories
    const auto debug_eb_traj = createTrajectory(p.path.header, debug_data_.eb_traj);
    debug_eb_traj_pub_->publish(debug_eb_traj);
  }

  {  // publish markers
    if (enable_debug_marker_pub_) {
      stop_watch_.tic("getDebugVisualizationMarker");
      const auto & debug_marker =
        debug_utils::getDebugVisualizationMarker(debug_data_, traj_points, vehicle_param_, false);
      debug_data_.msg_stream << "      getDebugVisualizationMarker:= "
                             << stop_watch_.toc("getDebugVisualizationMarker") << " [ms]\n";

      stop_watch_.tic("publishDebugVisualizationMarker");
      debug_markers_pub_->publish(debug_marker);
      debug_data_.msg_stream << "      publishDebugVisualizationMarker:= "
                             << stop_watch_.toc("publishDebugVisualizationMarker") << " [ms]\n";

      stop_watch_.tic("getDebugVisualizationWallMarker");
      const auto & debug_wall_marker =
        debug_utils::getDebugVisualizationWallMarker(debug_data_, vehicle_param_);
      debug_data_.msg_stream << "      getDebugVisualizationWallMarker:= "
                             << stop_watch_.toc("getDebugVisualizationWallMarker") << " [ms]\n";

      stop_watch_.tic("publishDebugVisualizationWallMarker");
      debug_wall_markers_pub_->publish(debug_wall_marker);
      debug_data_.msg_stream << "      publishDebugVisualizationWallMarker:= "
                             << stop_watch_.toc("publishDebugVisualizationWallMarker") << " [ms]\n";
    }
  }

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::generatePostProcessedTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & optimized_traj_points)
{
  stop_watch_.tic(__func__);

  const auto & p = planner_data;

  if (p.path.points.empty()) {
    return std::vector<TrajectoryPoint>{};
  }
  if (optimized_traj_points.empty()) {
    return points_utils::convertToTrajectoryPoints(p.path.points);
  }

  // calculate extended trajectory that connects to optimized trajectory smoothly
  auto full_traj_points = extendTrajectory(p.path.points, optimized_traj_points);

  // insert zero velocity after stop point
  // NOTE: This implementation is required for stopping due to going outside the drivable area.
  const auto opt_zero_vel_idx = motion_utils::searchZeroVelocityIndex(full_traj_points);
  if (opt_zero_vel_idx) {
    for (size_t i = opt_zero_vel_idx.get(); i < full_traj_points.size(); ++i) {
      full_traj_points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }

  /*
  // concat trajectories
  const auto full_traj_points = concatTrajectory(optimized_traj_points, extended_traj_points);

  // NOTE: fine_traj_points has no velocity information
  const auto fine_traj_points = generateFineTrajectoryPoints(path_points, full_traj_points);

  const auto fine_traj_points_with_vel =
    alignVelocity(fine_traj_points, path_points, full_traj_points);
  */

  debug_data_.msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return full_traj_points;
}

std::vector<TrajectoryPoint> CollisionFreePathPlanner::extendTrajectory(
  const std::vector<PathPoint> & path_points, const std::vector<TrajectoryPoint> & optimized_points)
{
  stop_watch_.tic(__func__);

  assert(!path_points.empty());

  /*
  const double accum_arc_length = motion_utils::calcArcLength(optimized_points);
  if (accum_arc_length > traj_param_.output_traj_length) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "[Avoidance] Not extend trajectory");
    return std::vector<TrajectoryPoint>{};
  }
  */

  // calculate end idx of optimized points on path points
  const auto opt_end_path_seg_idx = motion_utils::findNearestSegmentIndex(
    path_points, optimized_points.back().pose, std::numeric_limits<double>::max(),
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (!opt_end_path_seg_idx) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::seconds(5).count(),
      "[Avoidance] Not extend trajectory since could not find nearest idx from last opt point");
    return std::vector<TrajectoryPoint>{};
  }
  const size_t end_path_seg_idx = opt_end_path_seg_idx.get();

  auto extended_traj_points = optimized_points;
  const size_t extended_start_point_idx = end_path_seg_idx + 1;
  for (size_t i = extended_start_point_idx; i < path_points.size(); ++i) {
    const auto extended_traj_point = points_utils::convertToTrajectoryPoint(path_points.at(i));
    extended_traj_points.push_back(extended_traj_point);
  }

  // TODO(murooka) resample

  // TODO(murooka) update vleocity on joint

  // TODO(murooka) compensate goal pose

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return extended_traj_points;
}

/*
std::vector<TrajectoryPoint> CollisionFreePathPlanner::alignVelocity(
  const std::vector<TrajectoryPoint> & fine_traj_points, const std::vector<PathPoint> & path_points,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  stop_watch_.tic(__func__);

  // insert zero velocity path index, and get optional zero_vel_path_idx
  const auto path_zero_vel_info =
    [&]() -> std::pair<std::vector<TrajectoryPoint>, boost::optional<size_t>> {
    const auto opt_path_zero_vel_idx = motion_utils::searchZeroVelocityIndex(path_points);
    if (opt_path_zero_vel_idx) {
      const auto & zero_vel_path_point = path_points.at(opt_path_zero_vel_idx.get());
      const auto opt_traj_seg_idx = motion_utils::findNearestSegmentIndex(
        fine_traj_points, zero_vel_path_point.pose, std::numeric_limits<double>::max(),
        traj_param_.delta_yaw_threshold_for_closest_point);
      if (opt_traj_seg_idx) {
        const auto interpolated_pose =
          lerpPose(fine_traj_points, zero_vel_path_point.pose.position, opt_traj_seg_idx.get());
        if (interpolated_pose) {
          TrajectoryPoint zero_vel_traj_point;
          zero_vel_traj_point.pose = interpolated_pose.get();
          zero_vel_traj_point.longitudinal_velocity_mps =
            zero_vel_path_point.longitudinal_velocity_mps;

          if (
            tier4_autoware_utils::calcDistance2d(
              fine_traj_points.at(opt_traj_seg_idx.get()).pose, zero_vel_traj_point.pose) < 1e-3) {
            return {fine_traj_points, opt_traj_seg_idx.get()};
          } else if (
            tier4_autoware_utils::calcDistance2d(
              fine_traj_points.at(opt_traj_seg_idx.get() + 1).pose, zero_vel_traj_point.pose) <
            1e-3) {
            return {fine_traj_points, opt_traj_seg_idx.get() + 1};
          }

          auto fine_traj_points_with_zero_vel = fine_traj_points;
          fine_traj_points_with_zero_vel.insert(
            fine_traj_points_with_zero_vel.begin() + opt_traj_seg_idx.get() + 1,
            zero_vel_traj_point);
          return {fine_traj_points_with_zero_vel, opt_traj_seg_idx.get() + 1};
        }
      }
    }

    return {fine_traj_points, {}};
  }();
  const auto fine_traj_points_with_path_zero_vel = path_zero_vel_info.first;
  const auto opt_zero_vel_path_idx = path_zero_vel_info.second;

  // search zero velocity index of fine_traj_points
  const size_t zero_vel_fine_traj_idx = [&]() {
    // zero velocity for being outside drivable area
    const size_t zero_vel_traj_idx = searchExtendedZeroVelocityIndex(
      fine_traj_points_with_path_zero_vel, traj_points,
      traj_param_.delta_yaw_threshold_for_closest_point);

    // zero velocity in path points
    if (opt_zero_vel_path_idx) {
      return std::min(opt_zero_vel_path_idx.get(), zero_vel_traj_idx);
    }
    return zero_vel_traj_idx;
  }();

  // interpolate z and velocity
  auto fine_traj_points_with_vel = fine_traj_points_with_path_zero_vel;
  size_t prev_begin_idx = 0;
  for (size_t i = 0; i < fine_traj_points_with_vel.size(); ++i) {
    auto truncated_points = points_utils::clipForwardPoints(path_points, prev_begin_idx, 5.0);
    if (truncated_points.size() < 2) {
      // NOTE: At least, two points must be contained in truncated_points
      truncated_points = std::vector<PathPoint>(
        path_points.begin() + prev_begin_idx,
        path_points.begin() + std::min(path_points.size(), prev_begin_idx + 2));
    }

    const auto & target_pose = fine_traj_points_with_vel[i].pose;
    const size_t closest_seg_idx = findNearestSegmentIndexWithSoftYawConstraints(
      truncated_points, target_pose, traj_param_.delta_dist_threshold_for_closest_point,
      traj_param_.delta_yaw_threshold_for_closest_point);

    // lerp z
    fine_traj_points_with_vel[i].pose.position.z =
      lerpPoseZ(truncated_points, target_pose.position, closest_seg_idx);

    // lerp vx
    const double target_vel = lerpTwistX(truncated_points, target_pose.position, closest_seg_idx);

    if (i >= zero_vel_fine_traj_idx) {
      fine_traj_points_with_vel[i].longitudinal_velocity_mps = 0.0;
    } else {
      fine_traj_points_with_vel[i].longitudinal_velocity_mps = target_vel;
    }

    // NOTE: closest_seg_idx is for the clipped trajectory. This operation must be "+=".
    prev_begin_idx += closest_seg_idx;
  }

  debug_data_.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return fine_traj_points_with_vel;
}
*/

void CollisionFreePathPlanner::publishDebugDataInMain(const Path & path) const
{
  stop_watch_.tic(__func__);

  {  // publish trajectories
    const auto debug_extended_fixed_traj =
      createTrajectory(path.header, debug_data_.extended_fixed_traj);
    debug_extended_fixed_traj_pub_->publish(debug_extended_fixed_traj);

    const auto debug_extended_non_fixed_traj =
      createTrajectory(path.header, debug_data_.extended_non_fixed_traj);
    debug_extended_non_fixed_traj_pub_->publish(debug_extended_non_fixed_traj);
  }

  {  // publish clearance map
    stop_watch_.tic("publishClearanceMap");

    if (enable_area_with_objects_pub_) {  // false by default
      debug_area_with_objects_pub_->publish(
        debug_utils::getDebugCostmap(debug_data_.area_with_objects_map, path.drivable_area));
    }
    if (enable_object_clearance_map_pub_) {  // false by default
      debug_object_clearance_map_pub_->publish(
        debug_utils::getDebugCostmap(debug_data_.only_object_clearance_map, path.drivable_area));
    }
    if (enable_clearance_map_pub_) {  // true by default
      debug_clearance_map_pub_->publish(
        debug_utils::getDebugCostmap(debug_data_.clearance_map, path.drivable_area));
    }
    debug_data_.msg_stream << "    getDebugCostMap * 3:= " << stop_watch_.toc("publishClearanceMap")
                           << " [ms]\n";
  }

  debug_data_.msg_stream << "  " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
}
}  // namespace collision_free_path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(collision_free_path_planner::CollisionFreePathPlanner)
