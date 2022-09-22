// Copyright 2022 Tier IV, Inc.
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

#include "sampler_node/trajectory_sampler_node.hpp"

#include "Eigen/src/Core/Matrix.h"
#include "frenet_planner/structures.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "sampler_common/constraints/path_footprint.hpp"
#include "sampler_common/constraints/soft_constraint.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/trajectory_reuse.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/debug.hpp"
#include "sampler_node/plot/debug_window.hpp"
#include "sampler_node/prepare_inputs.hpp"
#include "sampler_node/trajectory_generation.hpp"
#include "sampler_node/utils/occupancy_grid_to_polygons.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace sampler_node
{
TrajectorySamplerNode::TrajectorySamplerNode(const rclcpp::NodeOptions & node_options)
: Node("trajectory_sampler_node", node_options),
  qapplication_(argc_, argv_.data()),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  w_.show();
  trajectory_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);

  path_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&TrajectorySamplerNode::pathCallback, this, std::placeholders::_1));
  // steer_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
  //   "~/input/steer", rclcpp::QoS{1},
  //   std::bind(&TrajectorySamplerNode::steerCallback, this, std::placeholders::_1));
  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&TrajectorySamplerNode::objectsCallback, this, std::placeholders::_1));
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectorySamplerNode::mapCallback, this, std::placeholders::_1));
  route_sub_ = create_subscription<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectorySamplerNode::routeCallback, this, std::placeholders::_1));
  fallback_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/fallback", rclcpp::QoS{1},
    std::bind(&TrajectorySamplerNode::fallbackCallback, this, std::placeholders::_1));
  vel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/velocity", rclcpp::QoS{1}, [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      current_velocity_ = msg->twist.twist.linear.x;
    });
  acc_sub_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/acceleration", rclcpp::QoS{1},
    [&](geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg) {
      current_acceleration_ = msg->accel.accel.linear.x;
    });

  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>();

  fallback_timeout_ = declare_parameter<double>("fallback_trajectory_timeout");
  params_.constraints.hard.max_curvature =
    declare_parameter<double>("constraints.hard.max_curvature");
  params_.constraints.hard.min_curvature =
    declare_parameter<double>("constraints.hard.min_curvature");
  params_.constraints.hard.max_acceleration =
    declare_parameter<double>("constraints.hard.max_acceleration");
  params_.constraints.hard.min_acceleration =
    declare_parameter<double>("constraints.hard.min_acceleration");
  params_.constraints.soft.lateral_deviation_weight =
    declare_parameter<double>("constraints.soft.lateral_deviation_weight");
  params_.constraints.soft.longitudinal_deviation_weight =
    declare_parameter<double>("constraints.soft.longitudinal_deviation_weight");
  params_.constraints.soft.jerk_weight = declare_parameter<double>("constraints.soft.jerk_weight");
  params_.constraints.soft.length_weight =
    declare_parameter<double>("constraints.soft.length_weight");
  params_.constraints.soft.curvature_weight =
    declare_parameter<double>("constraints.soft.curvature_weight");
  params_.sampling.enable_frenet = declare_parameter<bool>("sampling.enable_frenet");
  params_.sampling.enable_bezier = declare_parameter<bool>("sampling.enable_bezier");
  params_.sampling.resolution = declare_parameter<double>("sampling.resolution");
  params_.sampling.minimum_committed_length =
    declare_parameter<double>("sampling.minimum_committed_length");
  params_.sampling.reuse_max_length_max =
    declare_parameter<double>("sampling.reuse_max_length_max");
  params_.sampling.reuse_samples = declare_parameter<int>("sampling.reuse_samples");
  params_.sampling.reuse_max_deviation = declare_parameter<double>("sampling.reuse_max_deviation");
  params_.sampling.target_lengths =
    declare_parameter<std::vector<double>>("sampling.target_lengths");
  params_.sampling.frenet.target_lateral_positions =
    declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_positions");
  params_.sampling.frenet.target_lateral_velocities =
    declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_velocities");
  params_.sampling.frenet.target_lateral_accelerations =
    declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_accelerations");
  params_.sampling.bezier.nb_k = declare_parameter<int>("sampling.bezier.nb_k");
  params_.sampling.bezier.mk_min = declare_parameter<double>("sampling.bezier.mk_min");
  params_.sampling.bezier.mk_max = declare_parameter<double>("sampling.bezier.mk_max");
  params_.sampling.bezier.nb_t = declare_parameter<int>("sampling.bezier.nb_t");
  params_.sampling.bezier.mt_min = declare_parameter<double>("sampling.bezier.mt_min");
  params_.sampling.bezier.mt_max = declare_parameter<double>("sampling.bezier.mt_max");
  params_.preprocessing.force_zero_deviation =
    declare_parameter<bool>("preprocessing.force_zero_initial_deviation");
  params_.preprocessing.force_zero_heading =
    declare_parameter<bool>("preprocessing.force_zero_initial_heading");
  params_.preprocessing.smooth_reference =
    declare_parameter<bool>("preprocessing.smooth_reference_trajectory");
  params_.postprocessing.desired_traj_behind_length =
    declare_parameter<double>("postprocessing.desired_traj_behind_length");

  // const auto half_wheel_tread = vehicle_info_.wheel_tread_m / 2.0;
  const auto left_offset = vehicle_info_.vehicle_width_m / 2.0;
  const auto right_offset = -vehicle_info_.vehicle_width_m / 2.0;
  // const auto right_offset = -(half_wheel_tread + vehicle_info_.right_overhang_m);
  const auto rear_offset = -vehicle_info_.rear_overhang_m;
  const auto front_offset = vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m;
  params_.constraints.vehicle_offsets.left_rear = Eigen::Vector2d(rear_offset, left_offset);
  params_.constraints.vehicle_offsets.left_front = Eigen::Vector2d(front_offset, left_offset);
  params_.constraints.vehicle_offsets.right_rear = Eigen::Vector2d(rear_offset, right_offset);
  params_.constraints.vehicle_offsets.right_front = Eigen::Vector2d(front_offset, right_offset);

  set_param_res_ =
    add_on_set_parameters_callback([this](const auto & params) { return onParameter(params); });
  // This is necessary to interact with the GUI even when we are not generating trajectories
  gui_process_timer_ =
    create_wall_timer(std::chrono::milliseconds(100), []() { QCoreApplication::processEvents(); });
}

rcl_interfaces::msg::SetParametersResult TrajectorySamplerNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "fallback_trajectory_timeout") {
      fallback_timeout_ = parameter.as_double();
    } else if (parameter.get_name() == "constraints.hard.max_curvature") {
      params_.constraints.hard.max_curvature = parameter.as_double();
    } else if (parameter.get_name() == "constraints.hard.min_curvature") {
      params_.constraints.hard.min_curvature = parameter.as_double();
    } else if (parameter.get_name() == "constraints.hard.max_acceleration") {
      params_.constraints.hard.max_acceleration = parameter.as_double();
    } else if (parameter.get_name() == "constraints.hard.min_acceleration") {
      params_.constraints.hard.min_acceleration = parameter.as_double();
    } else if (parameter.get_name() == "constraints.soft.lateral_deviation_weight") {
      params_.constraints.soft.lateral_deviation_weight = parameter.as_double();
    } else if (parameter.get_name() == "constraints.soft.longitudinal_deviation_weight") {
      params_.constraints.soft.longitudinal_deviation_weight = parameter.as_double();
    } else if (parameter.get_name() == "constraints.soft.jerk_weight") {
      params_.constraints.soft.jerk_weight = parameter.as_double();
    } else if (parameter.get_name() == "constraints.soft.length_weight") {
      params_.constraints.soft.length_weight = parameter.as_double();
    } else if (parameter.get_name() == "constraints.soft.curvature_weight") {
      params_.constraints.soft.curvature_weight = parameter.as_double();
    } else if (parameter.get_name() == "sampling.enable_frenet") {
      params_.sampling.enable_frenet = parameter.as_bool();
    } else if (parameter.get_name() == "sampling.enable_bezier") {
      params_.sampling.enable_bezier = parameter.as_bool();
    } else if (parameter.get_name() == "sampling.resolution") {
      params_.sampling.resolution = parameter.as_double();
    } else if (parameter.get_name() == "sampling.minimum_committed_length") {
      params_.sampling.minimum_committed_length = parameter.as_double();
    } else if (parameter.get_name() == "sampling.reuse_max_length_max") {
      params_.sampling.reuse_max_length_max = parameter.as_double();
    } else if (parameter.get_name() == "sampling.reuse_samples") {
      params_.sampling.reuse_samples = parameter.as_int();
    } else if (parameter.get_name() == "sampling.reuse_max_deviation") {
      params_.sampling.reuse_max_deviation = parameter.as_double();
    } else if (parameter.get_name() == "sampling.target_lengths") {
      params_.sampling.target_lengths = parameter.as_double_array();
    } else if (parameter.get_name() == "sampling.frenet.target_lateral_positions") {
      params_.sampling.frenet.target_lateral_positions = parameter.as_double_array();
    } else if (parameter.get_name() == "sampling.frenet.target_lateral_velocities") {
      params_.sampling.frenet.target_lateral_velocities = parameter.as_double_array();
    } else if (parameter.get_name() == "sampling.frenet.target_lateral_accelerations") {
      params_.sampling.frenet.target_lateral_accelerations = parameter.as_double_array();
    } else if (parameter.get_name() == "sampling.bezier.nb_k") {
      params_.sampling.bezier.nb_k = parameter.as_int();
    } else if (parameter.get_name() == "sampling.bezier.mk_min") {
      params_.sampling.bezier.mk_min = parameter.as_double();
    } else if (parameter.get_name() == "sampling.bezier.mk_max") {
      params_.sampling.bezier.mk_max = parameter.as_double();
    } else if (parameter.get_name() == "sampling.bezier.nb_t") {
      params_.sampling.bezier.nb_t = parameter.as_int();
    } else if (parameter.get_name() == "sampling.bezier.mt_min") {
      params_.sampling.bezier.mt_min = parameter.as_double();
    } else if (parameter.get_name() == "sampling.bezier.mt_max") {
      params_.sampling.bezier.mt_max = parameter.as_double();
    } else if (parameter.get_name() == "preprocessing.force_zero_initial_deviation") {
      params_.preprocessing.force_zero_deviation = parameter.as_bool();
    } else if (parameter.get_name() == "preprocessing.force_zero_initial_heading") {
      params_.preprocessing.force_zero_heading = parameter.as_bool();
    } else if (parameter.get_name() == "preprocessing.smooth_reference_trajectory") {
      params_.preprocessing.smooth_reference = parameter.as_bool();
    } else if (parameter.get_name() == "postprocessing.desired_traj_behind_length") {
      params_.postprocessing.desired_traj_behind_length = parameter.as_double();
    } else {
      RCLCPP_WARN(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

// ROS callback functions
void TrajectorySamplerNode::pathCallback(
  const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg)
{
  w_.plotter_->clear();
  sampler_node::debug::Debug debug;
  const auto current_state = getCurrentEgoConfiguration();
  // TODO(Maxime CLEMENT): move to "validInputs(current_state, msg)"
  if (msg->points.size() < 2 || msg->drivable_area.data.empty() || !current_state) {
    RCLCPP_INFO(
      get_logger(),
      "[pathCallback] incomplete inputs: current_state: %d | drivable_area: %d | path points: %ld",
      current_state.has_value(), !msg->drivable_area.data.empty(), msg->points.size());
    return;
  }

  const auto calc_begin = std::chrono::steady_clock::now();

  const auto path_spline = preparePathSpline(*msg, params_.preprocessing.smooth_reference);
  const auto planning_configuration = getPlanningConfiguration(*current_state, path_spline);
  prepareConstraints(
    params_.constraints, *in_objects_ptr_, *lanelet_map_ptr_, drivable_ids_, prefered_ids_,
    msg->drivable_area);

  auto trajectories = generateCandidateTrajectories(
    planning_configuration, prev_traj_, path_spline, *msg, *w_.plotter_, params_);
  for (auto & trajectory : trajectories) {
    debug.violations +=
      sampler_common::constraints::checkHardConstraints(trajectory, params_.constraints);
    sampler_common::constraints::calculateCost(trajectory, params_.constraints, path_spline);
  }
  auto selected_trajectory = selectBestTrajectory(trajectories);
  if (selected_trajectory) {
    auto final_trajectory = prependTrajectory(*selected_trajectory, path_spline);
    publishTrajectory(final_trajectory, msg->header.frame_id);
    prev_traj_ = *selected_trajectory;
  } else {
    RCLCPP_DEBUG(
      get_logger(), "All candidates rejected: out=%d coll=%d curv=%d vel=%d acc=%d",
      debug.violations.outside, debug.violations.collision, debug.violations.curvature,
      debug.violations.velocity, debug.violations.acceleration);
    if (
      fallback_traj_ptr_ &&
      (now() - fallback_traj_ptr_->header.stamp).seconds() < fallback_timeout_) {
      RCLCPP_WARN(get_logger(), "Using fallback trajectory");
      trajectory_pub_->publish(*fallback_traj_ptr_);
      prev_traj_.clear();
    } else {
      publishTrajectory(prev_traj_, msg->header.frame_id);
    }
  }
  std::chrono::steady_clock::time_point calc_end = std::chrono::steady_clock::now();

  // Plot //
  const auto plot_begin = calc_end;
  w_.plotter_->plotPolygons(params_.constraints.obstacle_polygons);
  w_.plotter_->plotPolygons(params_.constraints.drivable_polygons);
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(msg->points.size());
  y.reserve(msg->points.size());
  for (const auto & p : msg->points) {
    x.push_back(p.pose.position.x);
    y.push_back(p.pose.position.y);
  }
  w_.plotter_->plotReferencePath(x, y);
  /*
  w_.plotter_->plotPaths(paths);
  if (selected_trajectory) {
    w_.plotter_->plotSelectedPath(*selected_trajectory);
    w_.plotter_->plotPolygons(
      {sampler_common::constraints::buildFootprintPolygon(*selected_trajectory,
  params_.constraints)});
  }
  */
  w_.plotter_->plotCurrentPose(path_spline.frenet(current_state->pose), current_state->pose);
  w_.replot();
  QCoreApplication::processEvents();
  w_.update();
  std::chrono::steady_clock::time_point plot_end = std::chrono::steady_clock::now();
  w_.setStatus(
    trajectories.size(),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(calc_end - calc_begin).count()),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_begin).count()));
}

std::optional<sampler_common::Trajectory> TrajectorySamplerNode::selectBestTrajectory(
  const std::vector<sampler_common::Trajectory> & trajectories)
{
  auto min_cost = std::numeric_limits<double>::max();
  std::optional<sampler_common::Trajectory> best_trajectory;
  for (const auto & traj : trajectories) {
    if (traj.valid && traj.cost < min_cost) {
      min_cost = traj.cost;
      best_trajectory = traj;
    }
  }
  return best_trajectory;
}

std::optional<sampler_common::Configuration> TrajectorySamplerNode::getCurrentEgoConfiguration()
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer_.lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "[TrajectorySamplerNode] %s", ex.what());
    return {};
  }

  auto config = sampler_common::Configuration();
  config.pose = {tf_current_pose.transform.translation.x, tf_current_pose.transform.translation.y};
  config.heading = tf2::getYaw(tf_current_pose.transform.rotation);
  config.velocity = current_velocity_;
  config.acceleration = current_acceleration_;
  return config;
}

void TrajectorySamplerNode::objectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>(*msg);
}

void TrajectorySamplerNode::publishTrajectory(
  const sampler_common::Trajectory & trajectory, const std::string & frame_id)
{
  tf2::Quaternion q;  // to convert yaw angle to Quaternion orientation
  autoware_auto_planning_msgs::msg::Trajectory traj_msg;
  traj_msg.header.frame_id = frame_id;
  traj_msg.header.stamp = now();
  autoware_auto_planning_msgs::msg::TrajectoryPoint point;
  for (size_t i = 0; i + 2 < trajectory.points.size(); ++i) {
    point.pose.position.x = trajectory.points[i].x();
    point.pose.position.y = trajectory.points[i].y();
    q.setRPY(0, 0, trajectory.yaws[i]);
    point.pose.orientation = tf2::toMsg(q);
    point.longitudinal_velocity_mps = static_cast<float>(trajectory.longitudinal_velocities[i]);
    point.acceleration_mps2 = static_cast<float>(trajectory.longitudinal_accelerations[i]);
    point.lateral_velocity_mps = static_cast<float>(trajectory.lateral_velocities[i]);
    point.heading_rate_rps =
      static_cast<float>(point.longitudinal_velocity_mps * trajectory.curvatures[i]);
    point.front_wheel_angle_rad = 0.0f;
    point.rear_wheel_angle_rad = 0.0f;
    traj_msg.points.push_back(point);
  }
  trajectory_pub_->publish(traj_msg);
}

// TODO(Maxime CLEMENT): unused in favor of the Path's drivable area
void TrajectorySamplerNode::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
}

// TODO(Maxime CLEMENT): unused in favor of the Path's drivable area
void TrajectorySamplerNode::routeCallback(
  const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr route_msg)
{
  prefered_ids_.clear();
  drivable_ids_.clear();
  if (lanelet_map_ptr_) {
    for (const auto & segment : route_msg->segments) {
      prefered_ids_.push_back(segment.preferred_primitive_id);
      for (const auto & primitive : segment.primitives) {
        drivable_ids_.push_back(primitive.id);
      }
    }
  }
}

void TrajectorySamplerNode::fallbackCallback(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr fallback_msg)
{
  fallback_traj_ptr_ = fallback_msg;
}

sampler_common::Configuration TrajectorySamplerNode::getPlanningConfiguration(
  sampler_common::Configuration state,
  const sampler_common::transform::Spline2D & path_spline) const
{
  const auto current_frenet = path_spline.frenet(state.pose);
  if (params_.preprocessing.force_zero_deviation) {
    state.pose = path_spline.cartesian(current_frenet.s);
  }
  if (params_.preprocessing.force_zero_heading) {
    state.heading = path_spline.yaw(current_frenet.s);
  }
  state.curvature = path_spline.curvature(current_frenet.s);
  return state;
}

sampler_common::Trajectory TrajectorySamplerNode::prependTrajectory(
  const sampler_common::Trajectory & trajectory,
  const sampler_common::transform::Spline2D & reference) const
{
  if (trajectory.points.empty()) return {};
  const auto current_frenet = reference.frenet(trajectory.points.front());
  const auto resolution = params_.sampling.resolution;
  sampler_common::Trajectory trajectory_to_prepend;
  const auto first_s = current_frenet.s - params_.postprocessing.desired_traj_behind_length;
  std::vector<double> ss;
  for (auto s = std::max(resolution, first_s); s < current_frenet.s; s += resolution)
    ss.push_back(s);
  for (const auto s : ss) {
    trajectory_to_prepend.points.push_back(reference.cartesian(s));
    trajectory_to_prepend.yaws.push_back(reference.yaw(s));
    trajectory_to_prepend.curvatures.push_back(reference.curvature(s));
    trajectory_to_prepend.intervals.push_back(resolution);
    trajectory_to_prepend.longitudinal_velocities.push_back(
      trajectory.longitudinal_velocities.front());
    trajectory_to_prepend.lateral_velocities.push_back(trajectory.lateral_velocities.front());
    trajectory_to_prepend.longitudinal_accelerations.push_back(
      trajectory.longitudinal_accelerations.front());
    trajectory_to_prepend.lateral_accelerations.push_back(trajectory.lateral_accelerations.front());
  }
  return trajectory_to_prepend.extend(trajectory);
}
}  // namespace sampler_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sampler_node::TrajectorySamplerNode)
