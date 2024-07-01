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

#include "autoware/control_evaluator/control_evaluator_node.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace control_diagnostics
{
controlEvaluatorNode::controlEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("control_evaluator", node_options)
{
  tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  using std::placeholders::_1;
  control_diag_sub_ = create_subscription<DiagnosticArray>(
    "~/input/diagnostics", 1, std::bind(&controlEvaluatorNode::onDiagnostics, this, _1));

  // Publisher
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);

  // Timer callback to publish evaluator diagnostics
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&controlEvaluatorNode::onTimer, this));
}

void controlEvaluatorNode::getRouteData()
{
  // route
  {
    const auto msg = route_subscriber_.takeNewData();
    if (msg) {
      if (msg->segments.empty()) {
        RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
      } else {
        route_handler_.setRoute(*msg);
        has_received_route_ = true;
      }
    }
  }
  // map
  {
    const auto msg = vector_map_subscriber_.takeNewData();
    if (msg) {
      route_handler_.setMap(*msg);
      has_received_map_ = true;
    }
  }
}

void controlEvaluatorNode::removeOldDiagnostics(const rclcpp::Time & stamp)
{
  constexpr double KEEP_TIME = 1.0;
  diag_queue_.erase(
    std::remove_if(
      diag_queue_.begin(), diag_queue_.end(),
      [stamp](const std::pair<diagnostic_msgs::msg::DiagnosticStatus, rclcpp::Time> & p) {
        return (stamp - p.second).seconds() > KEEP_TIME;
      }),
    diag_queue_.end());
}

void controlEvaluatorNode::removeDiagnosticsByName(const std::string & name)
{
  diag_queue_.erase(
    std::remove_if(
      diag_queue_.begin(), diag_queue_.end(),
      [&name](const std::pair<diagnostic_msgs::msg::DiagnosticStatus, rclcpp::Time> & p) {
        return p.first.name.find(name) != std::string::npos;
      }),
    diag_queue_.end());
}

void controlEvaluatorNode::addDiagnostic(
  const diagnostic_msgs::msg::DiagnosticStatus & diag, const rclcpp::Time & stamp)
{
  diag_queue_.push_back(std::make_pair(diag, stamp));
}

void controlEvaluatorNode::updateDiagnosticQueue(
  const DiagnosticArray & input_diagnostics, const std::string & function,
  const rclcpp::Time & stamp)
{
  const auto it = std::find_if(
    input_diagnostics.status.begin(), input_diagnostics.status.end(),
    [&function](const diagnostic_msgs::msg::DiagnosticStatus & diag) {
      return diag.name.find(function) != std::string::npos;
    });
  if (it != input_diagnostics.status.end()) {
    removeDiagnosticsByName(it->name);
    addDiagnostic(*it, input_diagnostics.header.stamp);
  }

  removeOldDiagnostics(stamp);
}

void controlEvaluatorNode::onDiagnostics(const DiagnosticArray::ConstSharedPtr diag_msg)
{
  // add target diagnostics to the queue and remove old ones
  for (const auto & function : target_functions_) {
    updateDiagnosticQueue(*diag_msg, function, now());
  }
}

DiagnosticStatus controlEvaluatorNode::generateAEBDiagnosticStatus(const DiagnosticStatus & diag)
{
  DiagnosticStatus status;
  status.level = status.OK;
  status.name = diag.name;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "decision";
  const bool is_emergency_brake = (diag.level == DiagnosticStatus::ERROR);
  key_value.value = (is_emergency_brake) ? "deceleration" : "none";
  status.values.push_back(key_value);
  return status;
}

DiagnosticStatus controlEvaluatorNode::generateLaneletDiagnosticStatus()
{
  DiagnosticStatus status;
  status.name = "ego_lane_info";

  getRouteData();
  if (!has_received_map_ || !has_received_route_) {
    status.level = status.ERROR;
    return status;
  }
  const auto current_lane = getCurrentLane();
  const auto ego_pose = getCurrentEgoPose();
  const lanelet::ConstLanelets current_lanelets{current_lane};
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(current_lanelets, ego_pose);
  status.level = status.OK;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "lane_id";
  key_value.value = std::to_string(current_lane.id());
  status.values.push_back(key_value);
  key_value.key = "s";
  key_value.value = std::to_string(arc_coordinates.length);
  status.values.push_back(key_value);
  key_value.key = "t";
  key_value.value = std::to_string(arc_coordinates.distance);
  status.values.push_back(key_value);
  return status;
}

DiagnosticStatus controlEvaluatorNode::generateLateralDeviationDiagnosticStatus(
  const Trajectory & traj, const Point & ego_point)
{
  const double lateral_deviation = metrics::calcLateralDeviation(traj, ego_point);

  DiagnosticStatus status;
  status.level = status.OK;
  status.name = "lateral_deviation";
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "metric_value";
  key_value.value = std::to_string(lateral_deviation);
  status.values.push_back(key_value);

  return status;
}

DiagnosticStatus controlEvaluatorNode::generateYawDeviationDiagnosticStatus(
  const Trajectory & traj, const Pose & ego_pose)
{
  const double yaw_deviation = metrics::calcYawDeviation(traj, ego_pose);

  DiagnosticStatus status;
  status.level = status.OK;
  status.name = "yaw_deviation";
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "metric_value";
  key_value.value = std::to_string(yaw_deviation);
  status.values.push_back(key_value);

  return status;
}

void controlEvaluatorNode::onTimer()
{
  DiagnosticArray metrics_msg;
  const auto traj = traj_sub_.takeData();
  const auto odom = odometry_sub_.takeData();

  // generate decision diagnostics from input diagnostics
  for (const auto & function : target_functions_) {
    const auto it = std::find_if(
      diag_queue_.begin(), diag_queue_.end(),
      [&function](const std::pair<diagnostic_msgs::msg::DiagnosticStatus, rclcpp::Time> & p) {
        return p.first.name.find(function) != std::string::npos;
      });
    if (it == diag_queue_.end()) {
      continue;
    }
    // generate each decision diagnostics
    // - AEB decision
    if (it->first.name.find("autonomous_emergency_braking") != std::string::npos) {
      metrics_msg.status.push_back(generateAEBDiagnosticStatus(it->first));
    }
  }

  // calculate deviation metrics
  if (odom && traj && !traj->points.empty()) {
    const Pose ego_pose = odom->pose.pose;
    metrics_msg.status.push_back(
      generateLateralDeviationDiagnosticStatus(*traj, ego_pose.position));
    metrics_msg.status.push_back(generateYawDeviationDiagnosticStatus(*traj, ego_pose));
  }

  metrics_msg.status.push_back(generateLaneletDiagnosticStatus());

  metrics_msg.header.stamp = now();
  metrics_pub_->publish(metrics_msg);
}

geometry_msgs::msg::Pose controlEvaluatorNode::getCurrentEgoPose() const
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  geometry_msgs::msg::Pose p;
  try {
    tf_current_pose = tf_buffer_ptr_->lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return p;
  }

  p.orientation = tf_current_pose.transform.rotation;
  p.position.x = tf_current_pose.transform.translation.x;
  p.position.y = tf_current_pose.transform.translation.y;
  p.position.z = tf_current_pose.transform.translation.z;

  return p;
}

lanelet::ConstLanelet controlEvaluatorNode::getCurrentLane()
{
  lanelet::ConstLanelet closest_lanelet;
  if (!has_received_map_ || !has_received_route_) {
    return closest_lanelet;
  }
  const auto ego_pose = getCurrentEgoPose();
  route_handler_.getClosestLaneletWithinRoute(ego_pose, &closest_lanelet);
  return closest_lanelet;
  // getRouteLanelets()
}

}  // namespace control_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(control_diagnostics::controlEvaluatorNode)
