// Copyright 2024 Tier IV, Inc.
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

#include "node.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/ros/wait_for_param.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware/universe_utils/transform/transforms.hpp>
#include <autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>
#include <autoware/velocity_smoother/trajectory_utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/time.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace
{
rclcpp::SubscriptionOptions create_subscription_options(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}
}  // namespace

namespace autoware::motion_velocity_planner
{
MotionVelocityPlannerNode::MotionVelocityPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("motion_velocity_planner_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  planner_data_(*this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Subscribers
  sub_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", 1, std::bind(&MotionVelocityPlannerNode::on_trajectory, this, _1),
    create_subscription_options(this));
  sub_lanelet_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS(10).transient_local(),
    std::bind(&MotionVelocityPlannerNode::on_lanelet_map, this, _1),
    create_subscription_options(this));

  srv_load_plugin_ = create_service<LoadPlugin>(
    "~/service/load_plugin", std::bind(&MotionVelocityPlannerNode::on_load_plugin, this, _1, _2));
  srv_unload_plugin_ = create_service<UnloadPlugin>(
    "~/service/unload_plugin",
    std::bind(&MotionVelocityPlannerNode::on_unload_plugin, this, _1, _2));

  // Publishers
  trajectory_pub_ =
    this->create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);
  velocity_limit_pub_ = this->create_publisher<VelocityLimit>(
    "~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
  clear_velocity_limit_pub_ = this->create_publisher<VelocityLimitClearCommand>(
    "~/output/clear_velocity_limit", rclcpp::QoS{1}.transient_local());
  processing_time_publisher_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  debug_viz_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);
  metrics_pub_ = this->create_publisher<MetricArray>("~/metrics", 1);

  // Parameters
  smooth_velocity_before_planning_ = declare_parameter<bool>("smooth_velocity_before_planning");
  // nearest search
  planner_data_.ego_nearest_dist_threshold =
    declare_parameter<double>("ego_nearest_dist_threshold");
  planner_data_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  planner_data_.trajectory_polygon_collision_check.decimate_trajectory_step_length =
    declare_parameter<double>("trajectory_polygon_collision_check.decimate_trajectory_step_length");
  planner_data_.trajectory_polygon_collision_check.goal_extended_trajectory_length =
    declare_parameter<double>("trajectory_polygon_collision_check.goal_extended_trajectory_length");
  planner_data_.trajectory_polygon_collision_check.enable_to_consider_current_pose =
    declare_parameter<bool>(
      "trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose");
  planner_data_.trajectory_polygon_collision_check.time_to_convergence = declare_parameter<double>(
    "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence");

  // set velocity smoother param
  set_velocity_smoother_params();

  // Initialize PlannerManager
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
    if (name.empty()) {
      break;
    }
    planner_manager_.load_module_plugin(*this, name);
  }

  set_param_callback_ = this->add_on_set_parameters_callback(
    std::bind(&MotionVelocityPlannerNode::on_set_param, this, std::placeholders::_1));

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
}

void MotionVelocityPlannerNode::on_load_plugin(
  const LoadPlugin::Request::SharedPtr request,
  [[maybe_unused]] const LoadPlugin::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.load_module_plugin(*this, request->plugin_name);
}

void MotionVelocityPlannerNode::on_unload_plugin(
  const UnloadPlugin::Request::SharedPtr request,
  [[maybe_unused]] const UnloadPlugin::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.unload_module_plugin(*this, request->plugin_name);
}

// NOTE: argument planner_data must not be referenced for multithreading
bool MotionVelocityPlannerNode::update_planner_data(
  std::map<std::string, double> & processing_times,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & input_traj_points)
{
  auto clock = *get_clock();
  auto is_ready = true;
  const auto check_with_log = [&](const auto ptr, const auto & log) {
    constexpr auto throttle_duration = 3000;  // [ms]
    if (!ptr) {
      RCLCPP_INFO_THROTTLE(get_logger(), clock, throttle_duration, "%s", log);
      is_ready = false;
      return false;
    }
    return true;
  };

  universe_utils::StopWatch<std::chrono::milliseconds> sw;
  const auto ego_state_ptr = sub_vehicle_odometry_.takeData();
  if (check_with_log(ego_state_ptr, "Waiting for current odometry"))
    planner_data_.current_odometry = *ego_state_ptr;
  processing_times["update_planner_data.odom"] = sw.toc(true);

  const auto ego_accel_ptr = sub_acceleration_.takeData();
  if (check_with_log(ego_accel_ptr, "Waiting for current acceleration"))
    planner_data_.current_acceleration = *ego_accel_ptr;
  processing_times["update_planner_data.accel"] = sw.toc(true);

  const auto predicted_objects_ptr = sub_predicted_objects_.takeData();
  if (check_with_log(predicted_objects_ptr, "Waiting for predicted objects"))
    planner_data_.process_predicted_objects(*predicted_objects_ptr);
  processing_times["update_planner_data.pred_obj"] = sw.toc(true);

  const auto no_ground_pointcloud_ptr = sub_no_ground_pointcloud_.takeData();
  if (check_with_log(no_ground_pointcloud_ptr, "Waiting for pointcloud")) {
    const auto no_ground_pointcloud = process_no_ground_pointcloud(no_ground_pointcloud_ptr);
    if (no_ground_pointcloud)
      planner_data_.no_ground_pointcloud = PlannerData::Pointcloud(*no_ground_pointcloud);
  }
  processing_times["update_planner_data.pcd"] = sw.toc(true);

  const auto occupancy_grid_ptr = sub_occupancy_grid_.takeData();
  if (check_with_log(occupancy_grid_ptr, "Waiting for the occupancy grid"))
    planner_data_.occupancy_grid = *occupancy_grid_ptr;
  processing_times["update_planner_data.occ_grid"] = sw.toc(true);

  // here we use bitwise operator to not short-circuit the logging messages
  is_ready &= check_with_log(map_ptr_, "Waiting for the map");
  processing_times["update_planner_data.map"] = sw.toc(true);
  is_ready &= check_with_log(
    planner_data_.velocity_smoother_, "Waiting for the initialization of the velocity smoother");
  processing_times["update_planner_data.smoother"] = sw.toc(true);

  // is driving forward
  const auto is_driving_forward =
    autoware::motion_utils::isDrivingForwardWithTwist(input_traj_points);
  if (is_driving_forward) {
    planner_data_.is_driving_forward = is_driving_forward.value();
  }
  processing_times["update_planner_data.is_driving_forward"] = sw.toc(true);

  // optional data
  const auto traffic_signals_ptr = sub_traffic_signals_.takeData();
  if (traffic_signals_ptr) process_traffic_signals(traffic_signals_ptr);
  processing_times["update_planner_data.traffic_lights"] = sw.toc(true);

  return is_ready;
}

std::optional<pcl::PointCloud<pcl::PointXYZ>>
MotionVelocityPlannerNode::process_no_ground_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return {};
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  if (!pc.empty()) autoware::universe_utils::transformPointCloud(pc, *pc_transformed, affine);
  return *pc_transformed;
}

void MotionVelocityPlannerNode::set_velocity_smoother_params()
{
  planner_data_.velocity_smoother_ =
    std::make_shared<autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother>(*this);
}

void MotionVelocityPlannerNode::on_lanelet_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  map_ptr_ = msg;
  has_received_map_ = true;
}

void MotionVelocityPlannerNode::process_traffic_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg)
{
  // clear previous observation
  planner_data_.traffic_light_id_map_raw_.clear();
  const auto traffic_light_id_map_last_observed_old =
    planner_data_.traffic_light_id_map_last_observed_;
  planner_data_.traffic_light_id_map_last_observed_.clear();
  for (const auto & signal : msg->traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg->stamp;
    traffic_signal.signal = signal;
    planner_data_.traffic_light_id_map_raw_[signal.traffic_light_group_id] = traffic_signal;
    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      });
    // if the observation is UNKNOWN and past observation is available, only update the timestamp
    // and keep the body of the info
    const auto old_data =
      traffic_light_id_map_last_observed_old.find(signal.traffic_light_group_id);
    if (is_unknown_observation && old_data != traffic_light_id_map_last_observed_old.end()) {
      // copy last observation
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        old_data->second;
      // update timestamp
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id].stamp =
        msg->stamp;
    } else {
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        traffic_signal;
    }
  }
}

void MotionVelocityPlannerNode::on_trajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_trajectory_msg)
{
  std::unique_lock<std::mutex> lk(mutex_);

  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  std::map<std::string, double> processing_times;
  stop_watch.tic("Total");

  if (!update_planner_data(processing_times, input_trajectory_msg->points)) {
    return;
  }
  processing_times["update_planner_data"] = stop_watch.toc(true);

  if (input_trajectory_msg->points.empty()) {
    RCLCPP_WARN(get_logger(), "Input trajectory message is empty");
    return;
  }

  if (has_received_map_) {
    planner_data_.route_handler = std::make_shared<route_handler::RouteHandler>(*map_ptr_);
    has_received_map_ = false;
    processing_times["make_RouteHandler"] = stop_watch.toc(true);
  }

  autoware::motion_velocity_planner::TrajectoryPoints input_trajectory_points{
    input_trajectory_msg->points.begin(), input_trajectory_msg->points.end()};
  auto output_trajectory_msg = generate_trajectory(input_trajectory_points, processing_times);
  output_trajectory_msg.header = input_trajectory_msg->header;
  processing_times["generate_trajectory"] = stop_watch.toc(true);

  lk.unlock();

  trajectory_pub_->publish(output_trajectory_msg);
  published_time_publisher_.publish_if_subscribed(
    trajectory_pub_, output_trajectory_msg.header.stamp);
  processing_times["Total"] = stop_watch.toc("Total");
  processing_diag_publisher_.publish(processing_times);
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = processing_times["Total"];
  processing_time_publisher_->publish(processing_time_msg);

  std::shared_ptr<MetricArray> metrics = planner_manager_.get_metrics(get_clock()->now());
  metrics_pub_->publish(*metrics);
  planner_manager_.clear_metrics();
}

void MotionVelocityPlannerNode::insert_stop(
  autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Point & stop_point) const
{
  const auto seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory.points, stop_point);
  const auto insert_idx =
    autoware::motion_utils::insertTargetPoint(seg_idx, stop_point, trajectory.points);
  if (insert_idx) {
    for (auto idx = *insert_idx; idx < trajectory.points.size(); ++idx)
      trajectory.points[idx].longitudinal_velocity_mps = 0.0;
  } else {
    RCLCPP_WARN(get_logger(), "Failed to insert stop point");
  }
}

void MotionVelocityPlannerNode::insert_slowdown(
  autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::motion_velocity_planner::SlowdownInterval & slowdown_interval) const
{
  const auto from_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory.points, slowdown_interval.from);
  const auto from_insert_idx = autoware::motion_utils::insertTargetPoint(
    from_seg_idx, slowdown_interval.from, trajectory.points);
  const auto to_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory.points, slowdown_interval.to);
  const auto to_insert_idx =
    autoware::motion_utils::insertTargetPoint(to_seg_idx, slowdown_interval.to, trajectory.points);
  if (from_insert_idx && to_insert_idx) {
    for (auto idx = *from_insert_idx; idx <= *to_insert_idx; ++idx) {
      trajectory.points[idx].longitudinal_velocity_mps =
        std::min(  // prevent the node from increasing the velocity
          trajectory.points[idx].longitudinal_velocity_mps,
          static_cast<float>(slowdown_interval.velocity));
    }
  } else {
    RCLCPP_WARN(get_logger(), "Failed to insert slowdown point");
  }
}

autoware::motion_velocity_planner::TrajectoryPoints MotionVelocityPlannerNode::smooth_trajectory(
  const autoware::motion_velocity_planner::TrajectoryPoints & trajectory_points,
  const autoware::motion_velocity_planner::PlannerData & planner_data) const
{
  const geometry_msgs::msg::Pose current_pose = planner_data.current_odometry.pose.pose;
  const double v0 = planner_data.current_odometry.twist.twist.linear.x;
  const double a0 = planner_data.current_acceleration.accel.accel.linear.x;
  const auto & smoother = planner_data.velocity_smoother_;

  const auto traj_lateral_acc_filtered =
    smoother->applyLateralAccelerationFilter(trajectory_points);

  const auto traj_steering_rate_limited =
    smoother->applySteeringRateLimit(traj_lateral_acc_filtered, false);

  // Resample trajectory with ego-velocity based interval distances
  auto traj_resampled = traj_steering_rate_limited;
  const size_t traj_resampled_closest =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_resampled, current_pose, planner_data.ego_nearest_dist_threshold,
      planner_data.ego_nearest_yaw_threshold);
  std::vector<autoware::motion_velocity_planner::TrajectoryPoints> debug_trajectories;
  // Clip trajectory from closest point
  autoware::motion_velocity_planner::TrajectoryPoints clipped;
  autoware::motion_velocity_planner::TrajectoryPoints traj_smoothed;
  clipped.insert(
    clipped.end(), std::next(traj_resampled.begin(), static_cast<int64_t>(traj_resampled_closest)),
    traj_resampled.end());
  if (!smoother->apply(v0, a0, clipped, traj_smoothed, debug_trajectories, false)) {
    RCLCPP_ERROR(get_logger(), "failed to smooth");
  }
  return traj_smoothed;
}

autoware_planning_msgs::msg::Trajectory MotionVelocityPlannerNode::generate_trajectory(
  const autoware::motion_velocity_planner::TrajectoryPoints & input_trajectory_points,
  std::map<std::string, double> & processing_times)
{
  universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  autoware_planning_msgs::msg::Trajectory output_trajectory_msg;
  output_trajectory_msg.points = {input_trajectory_points.begin(), input_trajectory_points.end()};

  stop_watch.tic("smooth");
  const auto smoothed_trajectory_points = [&]() {
    if (smooth_velocity_before_planning_) {
      return smooth_trajectory(input_trajectory_points, planner_data_);
    }
    return input_trajectory_points;
  }();
  processing_times["velocity_smoothing"] = stop_watch.toc("smooth");

  stop_watch.tic("resample");
  TrajectoryPoints resampled_smoothed_trajectory_points;
  // skip points that are too close together to make computation easier
  if (!smoothed_trajectory_points.empty()) {
    resampled_smoothed_trajectory_points.push_back(smoothed_trajectory_points.front());
    constexpr auto min_interval_squared = 0.5 * 0.5;  // TODO(Maxime): change to a parameter
    for (auto i = 1UL; i < smoothed_trajectory_points.size(); ++i) {
      const auto & p = smoothed_trajectory_points[i];
      const auto dist_to_prev_point =
        universe_utils::calcSquaredDistance2d(resampled_smoothed_trajectory_points.back(), p);
      if (dist_to_prev_point > min_interval_squared) {
        resampled_smoothed_trajectory_points.push_back(p);
      }
    }
  }
  processing_times["resample"] = stop_watch.toc("resample");
  stop_watch.tic("calculate_time_from_start");
  motion_utils::calculate_time_from_start(
    resampled_smoothed_trajectory_points, planner_data_.current_odometry.pose.pose.position);
  processing_times["calculate_time_from_start"] = stop_watch.toc("calculate_time_from_start");
  stop_watch.tic("plan_velocities");
  const auto planning_results = planner_manager_.plan_velocities(
    input_trajectory_points, resampled_smoothed_trajectory_points,
    std::make_shared<const PlannerData>(planner_data_));
  processing_times["plan_velocities"] = stop_watch.toc("plan_velocities");

  for (const auto & planning_result : planning_results) {
    for (const auto & stop_point : planning_result.stop_points)
      insert_stop(output_trajectory_msg, stop_point);
    for (const auto & slowdown_interval : planning_result.slowdown_intervals)
      insert_slowdown(output_trajectory_msg, slowdown_interval);
    if (planning_result.velocity_limit) {
      velocity_limit_pub_->publish(*planning_result.velocity_limit);
    }
    if (planning_result.velocity_limit_clear_command) {
      clear_velocity_limit_pub_->publish(*planning_result.velocity_limit_clear_command);
    }
  }

  return output_trajectory_msg;
}

rcl_interfaces::msg::SetParametersResult MotionVelocityPlannerNode::on_set_param(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;

  {
    std::unique_lock<std::mutex> lk(mutex_);  // for planner_manager_
    planner_manager_.update_module_parameters(parameters);
  }

  updateParam(parameters, "smooth_velocity_before_planning", smooth_velocity_before_planning_);
  updateParam(parameters, "ego_nearest_dist_threshold", planner_data_.ego_nearest_dist_threshold);
  updateParam(parameters, "ego_nearest_yaw_threshold", planner_data_.ego_nearest_yaw_threshold);
  updateParam(
    parameters, "trajectory_polygon_collision_check.decimate_trajectory_step_length",
    planner_data_.trajectory_polygon_collision_check.decimate_trajectory_step_length);
  updateParam(
    parameters, "trajectory_polygon_collision_check.goal_extended_trajectory_length",
    planner_data_.trajectory_polygon_collision_check.goal_extended_trajectory_length);
  updateParam(
    parameters,
    "trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose",
    planner_data_.trajectory_polygon_collision_check.enable_to_consider_current_pose);
  updateParam(
    parameters, "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence",
    planner_data_.trajectory_polygon_collision_check.time_to_convergence);

  // set_velocity_smoother_params(); TODO(Maxime): fix update parameters of the velocity smoother

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace autoware::motion_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion_velocity_planner::MotionVelocityPlannerNode)
