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

#include "sampler_node/path_sampler_node.hpp"

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
PathSamplerNode::PathSamplerNode(const rclcpp::NodeOptions & node_options)
: Node("path_sampler_node", node_options),
  qapplication_(argc_, argv_.data()),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  w_.show();
  trajectory_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);

  path_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&PathSamplerNode::pathCallback, this, std::placeholders::_1));
  steer_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "~/input/steer", rclcpp::QoS{1},
    std::bind(&PathSamplerNode::steerCallback, this, std::placeholders::_1));
  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&PathSamplerNode::objectsCallback, this, std::placeholders::_1));
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&PathSamplerNode::mapCallback, this, std::placeholders::_1));
  route_sub_ = create_subscription<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&PathSamplerNode::routeCallback, this, std::placeholders::_1));

  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>();

  params_.constraints.hard.max_curvature =
    declare_parameter<double>("constraints.hard.max_curvature");
  params_.constraints.hard.min_curvature =
    declare_parameter<double>("constraints.hard.min_curvature");
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

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  params_.constraints.vehicle_offsets.left_rear =
    Eigen::Vector2d(vehicle_info.rear_overhang_m, vehicle_info.left_overhang_m);
  params_.constraints.vehicle_offsets.left_front =
    Eigen::Vector2d(vehicle_info.front_overhang_m, vehicle_info.left_overhang_m);
  params_.constraints.vehicle_offsets.right_rear =
    Eigen::Vector2d(vehicle_info.rear_overhang_m, vehicle_info.right_overhang_m);
  params_.constraints.vehicle_offsets.right_front =
    Eigen::Vector2d(vehicle_info.front_overhang_m, vehicle_info.right_overhang_m);

  set_param_res_ =
    add_on_set_parameters_callback([this](const auto & params) { return onParameter(params); });
  // This is necessary to interact with the GUI even when we are not generating trajectories
  gui_process_timer_ =
    create_wall_timer(std::chrono::milliseconds(100), []() { QCoreApplication::processEvents(); });
}

rcl_interfaces::msg::SetParametersResult PathSamplerNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "constraints.hard.max_curvature") {
      params_.constraints.hard.max_curvature = parameter.as_double();
    } else if (parameter.get_name() == "constraints.hard.min_curvature") {
      params_.constraints.hard.min_curvature = parameter.as_double();
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
    } else {
      RCLCPP_WARN(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

// ROS callback functions
void PathSamplerNode::pathCallback(const autoware_auto_planning_msgs::msg::Path::SharedPtr msg)
{
  w_.plotter_->clear();
  sampler_node::debug::Debug debug;
  const auto current_state = getCurrentEgoState();
  // TODO(Maxime CLEMENT): move to "validInputs(current_state, msg)"
  if (
    msg->points.size() < 2 || msg->drivable_area.data.empty() || !current_state ||
    !current_steer_ptr_) {
    RCLCPP_INFO(
      get_logger(),
      "[pathCallback] incomplete inputs: current_state: %d | drivable_area: %d | path points: %ld",
      current_state.has_value(), !msg->drivable_area.data.empty(), msg->points.size());
    return;
  }

  // TODO(Maxime CLEMENT): use common StopWatch
  const auto calc_begin = std::chrono::steady_clock::now();

  const auto path_spline = preparePathSpline(*msg);
  prepareConstraints(
    params_.constraints, *in_objects_ptr_, *lanelet_map_ptr_, drivable_ids_, prefered_ids_);

  auto paths =
    generateCandidatePaths(*current_state, prev_path_, path_spline, *msg, *w_.plotter_, params_);
  for (auto & path : paths) {
    const auto nb_violations =
      sampler_common::constraints::checkHardConstraints(path, params_.constraints);
    debug.violations.outside += nb_violations.outside;
    debug.violations.collision += nb_violations.collision;
    debug.violations.curvature += nb_violations.curvature;
    sampler_common::constraints::calculateCost(path, params_.constraints, path_spline);
  }
  const auto selected_path = selectBestPath(paths);
  if (selected_path) {
    // combine newly computed trajectory with the base trajectory
    publishPath(*selected_path, msg);
    prev_path_ = *selected_path;
  } else {
    RCLCPP_WARN(
      get_logger(), "[PathSampler] All candidates rejected: out=%d coll=%d curv=%d",
      debug.violations.outside, debug.violations.collision, debug.violations.curvature);
    publishPath(prev_path_, msg);
  }
  std::chrono::steady_clock::time_point calc_end = std::chrono::steady_clock::now();

  // Plot //
  const auto plot_begin = calc_end;
  w_.plotter_->plotPolygons(params_.constraints.obstacle_polygons);
  w_.plotter_->plotPolygons(
    {params_.constraints.drivable_polygon, params_.constraints.prefered_polygon});
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(msg->points.size());
  y.reserve(msg->points.size());
  for (const auto & p : msg->points) {
    x.push_back(p.pose.position.x);
    y.push_back(p.pose.position.y);
  }
  w_.plotter_->plotReferencePath(x, y);
  w_.plotter_->plotPaths(paths);
  if (selected_path) w_.plotter_->plotSelectedPath(*selected_path);
  // TODO(Maxime CLEMENT): temporary
  if (selected_path)
    w_.plotter_->plotPolygons(
      {sampler_common::constraints::buildFootprintPolygon(*selected_path, params_.constraints)});

  w_.plotter_->plotCurrentPose(path_spline.frenet(current_state->pose), current_state->pose);
  w_.replot();
  QCoreApplication::processEvents();
  w_.update();
  std::chrono::steady_clock::time_point plot_end = std::chrono::steady_clock::now();
  w_.setStatus(
    paths.size(),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(calc_end - calc_begin).count()),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_begin).count()));
}

std::optional<sampler_common::Path> PathSamplerNode::selectBestPath(
  const std::vector<sampler_common::Path> & paths)
{
  auto min_cost = std::numeric_limits<double>::max();
  std::optional<sampler_common::Path> best_path;
  for (const auto & path : paths) {
    if (path.valid && path.cost < min_cost) {
      min_cost = path.cost;
      best_path = path;
    }
  }
  return best_path;
}

void PathSamplerNode::steerCallback(
  const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
{
  current_steer_ptr_ = msg;
}

std::optional<sampler_common::State> PathSamplerNode::getCurrentEgoState()
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer_.lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "[PathSamplerNode] %s", ex.what());
    return {};
  }

  auto state = sampler_common::State();
  state.pose = {tf_current_pose.transform.translation.x, tf_current_pose.transform.translation.y};
  state.heading = tf2::getYaw(tf_current_pose.transform.rotation);
  const auto wheel_base = 2.0;  // TODO(Maxime CLEMENT): get from parameter
  state.curvature = std::sin(current_steer_ptr_->steering_tire_angle) / wheel_base;
  return state;
}

void PathSamplerNode::objectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>(*msg);
}

void PathSamplerNode::publishPath(
  const sampler_common::Path & path,
  const autoware_auto_planning_msgs::msg::Path::SharedPtr path_msg)
{
  if (path.points.size() < 2) {
    return;
  }
  std::vector<double> velocities(path.intervals.size());
  double path_arc_length = 0.0;
  double msg_arc_length = 0.0;
  size_t msg_idx = 0;
  for (size_t i = 0; i < path.intervals.size(); ++i) {
    path_arc_length += path.intervals[i];
    while (msg_arc_length < path_arc_length && msg_idx + 1 < path_msg->points.size()) {
      const auto x0 = path_msg->points[msg_idx].pose.position.x;
      const auto y0 = path_msg->points[msg_idx].pose.position.y;
      const auto x1 = path_msg->points[msg_idx + 1].pose.position.x;
      const auto y1 = path_msg->points[msg_idx + 1].pose.position.y;
      msg_arc_length += std::hypot(x1 - x0, y1 - y0);
      ++msg_idx;
    }
    velocities[i] = path_msg->points[msg_idx].longitudinal_velocity_mps;
  }
  tf2::Quaternion q;  // to convert yaw angle to Quaternion orientation
  autoware_auto_planning_msgs::msg::Trajectory traj_msg;
  traj_msg.header.frame_id = path_msg->header.frame_id;
  traj_msg.header.stamp = now();
  for (size_t i = 0; i < path.points.size() - 2; ++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = path.points[i].x();
    point.pose.position.y = path.points[i].y();
    q.setRPY(0, 0, path.yaws[i]);
    point.pose.orientation = tf2::toMsg(q);
    point.longitudinal_velocity_mps = static_cast<float>(velocities[i]);
    point.front_wheel_angle_rad = static_cast<float>(path.curvatures[i]);
    point.rear_wheel_angle_rad = 0.0f;
    traj_msg.points.push_back(point);
  }
  trajectory_pub_->publish(traj_msg);
}

void PathSamplerNode::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::SharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
}
void PathSamplerNode::routeCallback(
  const autoware_auto_planning_msgs::msg::HADMapRoute::SharedPtr route_msg)
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
}  // namespace sampler_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sampler_node::PathSamplerNode)
