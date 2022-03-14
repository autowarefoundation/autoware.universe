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

#include "frenet_planner/structures.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "sampler_common/constraints/soft_constraint.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/trajectory_reuse.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/path_generation.hpp"
#include "sampler_node/plot/debug_window.hpp"
#include "sampler_node/prepare_inputs.hpp"
#include "sampler_node/utils/occupancy_grid_to_polygons.hpp"
#include "sampler_node/debug.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
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

  // This is necessary to interact with the GUI even when we are not generating trajectories
  gui_process_timer_ =
    create_wall_timer(std::chrono::milliseconds(100), []() { QCoreApplication::processEvents(); });
}

// ROS callback functions
void PathSamplerNode::pathCallback(const autoware_auto_planning_msgs::msg::Path::SharedPtr msg)
{
  w_.plotter_->clear();
  sampler_node::debug::Debug debug;
  const auto current_state = getCurrentEgoState();
  // TODO move to "validInputs(current_state, msg)"
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
  const auto constraints = prepareConstraints(*in_objects_ptr_, *lanelet_map_ptr_, drivable_ids_, prefered_ids_);

  auto paths = generateCandidatePaths(*current_state, prev_path_, path_spline, *msg, constraints, *w_.plotter_);
  for (auto & path : paths) {
    const auto nb_violations = sampler_common::constraints::checkHardConstraints(path, constraints);
    debug.violations.outside += nb_violations.outside;
    debug.violations.collision += nb_violations.collision;
    debug.violations.curvature += nb_violations.curvature;
    sampler_common::constraints::calculateCost(path, constraints, path_spline);
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
  w_.plotter_->plotPolygons(constraints.obstacle_polygons);
  w_.plotter_->plotPolygons(constraints.drivable_polygons);
  w_.plotter_->plotPolygons(constraints.prefered_polygons);
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
  for(size_t i = 0; i < path.intervals.size(); ++i) {
    path_arc_length += path.intervals[i];
    while(msg_arc_length < path_arc_length && msg_idx + 1 < path_msg->points.size()) {
      const auto x0 = path_msg->points[msg_idx].pose.position.x;
      const auto y0 = path_msg->points[msg_idx].pose.position.y;
      const auto x1 = path_msg->points[msg_idx+1].pose.position.x;
      const auto y1 = path_msg->points[msg_idx+1].pose.position.y;
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

void PathSamplerNode::mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::SharedPtr map_msg) {
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
}
void PathSamplerNode::routeCallback(const autoware_auto_planning_msgs::msg::HADMapRoute::SharedPtr route_msg) {
  prefered_ids_.clear();
  drivable_ids_.clear();
  if(lanelet_map_ptr_) {
    for(const auto & segment : route_msg->segments) {
      prefered_ids_.push_back(segment.preferred_primitive_id);
      for(const auto & primitive : segment.primitives) {
        drivable_ids_.push_back(primitive.id);
      }
    }
  }
}
}  // namespace sampler_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sampler_node::PathSamplerNode)
