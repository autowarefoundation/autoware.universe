// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include "autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
double calcReedsSheppDistance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, double radius)
{
  const auto rs_space = ReedsSheppStateSpace(radius);
  const ReedsSheppStateSpace::StateXYT pose0{
    p1.position.x, p1.position.y, tf2::getYaw(p1.orientation)};
  const ReedsSheppStateSpace::StateXYT pose1{
    p2.position.x, p2.position.y, tf2::getYaw(p2.orientation)};
  return rs_space.distance(pose0, pose1);
}

void setYaw(geometry_msgs::msg::Quaternion * orientation, const double yaw)
{
  *orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
}

geometry_msgs::msg::Pose calcRelativePose(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf_transform;
  tf2::convert(base_pose, tf_transform);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::msg::PoseStamped transformed;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed, transform);

  return transformed.pose;
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param)
: AbstractPlanningAlgorithm(planner_common_param, collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  search_method_ =
    astar_param.search_method == "backward" ? SearchMethod::Backward : SearchMethod::Forward;

  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = std::max(
    collision_vehicle_shape_.base_length * base_length_max_expansion_factor_, min_expansion_dist_);
}

bool AstarSearch::makePlan(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  if (search_method_ == SearchMethod::Backward) {
    start_pose_ = global2local(costmap_, goal_pose);
    goal_pose_ = global2local(costmap_, start_pose);
  } else {
    start_pose_ = global2local(costmap_, start_pose);
    goal_pose_ = global2local(costmap_, goal_pose);
  }

  resetData();

  if (!setStartNode()) {
    throw std::logic_error("Invalid start pose");
    return false;
  }

  if (!setGoalNode()) {
    throw std::logic_error("Invalid goal pose");
    return false;
  }

  if (!search()) {
    throw std::logic_error("HA* failed to find path to goal");
    return false;
  }
  return true;
}

void AstarSearch::resetData()
{
  // clearing openlist is necessary because otherwise remaining elements of openlist
  // point to deleted node.
  openlist_ = std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison>();
  graph_.clear();
  int total_astar_node_count =
    costmap_.info.width * costmap_.info.height * planner_common_param_.theta_size;
  graph_.resize(total_astar_node_count);
}

bool AstarSearch::setStartNode()
{
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);

  if (detectCollision(index)) {
    return false;
  }

  // Set start node
  AstarNode * start_node = &graph_[getKey(index)];
  start_node->x = start_pose_.position.x;
  start_node->y = start_pose_.position.y;
  start_node->theta = 2.0 * M_PI / planner_common_param_.theta_size * index.theta;
  start_node->gc = 0;
  start_node->fc = estimateCost(start_pose_);
  start_node->dist_to_goal = autoware::universe_utils::calcDistance2d(start_pose_, goal_pose_);
  start_node->dist_to_obs = getObstacleEDT(index);
  start_node->steering_index = 0;
  start_node->is_back = false;
  start_node->status = NodeStatus::Open;
  start_node->parent = nullptr;

  // Push start node to openlist
  openlist_.push(start_node);

  return true;
}

bool AstarSearch::setGoalNode()
{
  const auto index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);

  if (detectCollision(index)) {
    return false;
  }

  return true;
}

double AstarSearch::estimateCost(const geometry_msgs::msg::Pose & pose) const
{
  double total_cost = 0.0;
  // Temporarily, until reeds_shepp gets stable.
  if (use_reeds_shepp_) {
    total_cost += calcReedsSheppDistance(pose, goal_pose_, avg_turning_radius_) *
                  astar_param_.distance_heuristic_weight;
  } else {
    total_cost += autoware::universe_utils::calcDistance2d(pose, goal_pose_) *
                  astar_param_.distance_heuristic_weight;
  }
  return total_cost;
}

bool AstarSearch::search()
{
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  // Start A* search
  while (!openlist_.empty()) {
    // Check time and terminate if the search reaches the time limit
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;
    if (msec > planner_common_param_.time_limit) {
      return false;
    }

    // Expand minimum cost node
    AstarNode * current_node = openlist_.top();
    openlist_.pop();
    if (current_node->status == NodeStatus::Closed) continue;
    current_node->status = NodeStatus::Closed;

    if (isGoal(*current_node)) {
      goal_node_ = current_node;
      setPath(*current_node);
      return true;
    }

    expandNodes(*current_node);
    if (astar_param_.use_back) expandNodes(*current_node, true);
  }

  // Failed to find path
  return false;
}

void AstarSearch::expandNodes(AstarNode & current_node, const bool is_back)
{
  const auto current_pose = node2pose(current_node);
  double direction = is_back ? -1.0 : 1.0;
  if (search_method_ == SearchMethod::Backward) std::negate(direction);
  double distance = getExpansionDistance(current_node) * direction;
  int steering_index = -1 * planner_common_param_.turning_steps;
  for (; steering_index <= planner_common_param_.turning_steps; ++steering_index) {
    // skip expansion back to parent
    // skip expansion resulting in frequent direction change
    if (current_node.parent != nullptr && is_back != current_node.is_back) {
      if (
        steering_index == current_node.steering_index ||
        current_node.dir_distance < min_dir_change_dist_)
        continue;
    }

    const double steering = static_cast<double>(steering_index) * steering_resolution_;
    const auto next_pose = kinematic_bicycle_model::getPose(
      current_pose, collision_vehicle_shape_.base_length, steering, distance);
    const auto next_index = pose2index(costmap_, next_pose, planner_common_param_.theta_size);

    if (isOutOfRange(next_index)) continue;
    if (isObs(next_index)) continue;

    AstarNode * next_node = &graph_[getKey(next_index)];
    if (next_node->status == NodeStatus::Closed) continue;
    if (detectCollision(next_index)) continue;

    double distance_to_obs = getObstacleEDT(next_index);
    const bool is_direction_switch =
      (current_node.parent != nullptr) && (is_back != current_node.is_back);
    double weights_sum = 1.0;
    weights_sum += is_direction_switch ? planner_common_param_.direction_change_weight : 0.0;
    weights_sum += getSteeringCost(steering_index);
    weights_sum += getSteeringChangeCost(steering_index, current_node.steering_index);
    weights_sum += astar_param_.obstacle_distance_weight / (1.0 + distance_to_obs);

    weights_sum *= is_back ? planner_common_param_.reverse_weight : 1.0;

    double move_cost = current_node.gc + weights_sum * std::abs(distance);
    double total_cost = move_cost + estimateCost(next_pose);
    // Compare cost
    if (next_node->status == NodeStatus::None || next_node->fc > total_cost) {
      next_node->status = NodeStatus::Open;
      next_node->set(next_pose, move_cost, total_cost, steering_index, is_back);
      next_node->dir_distance =
        std::abs(distance) + (is_direction_switch ? 0.0 : current_node.dir_distance);
      next_node->dist_to_goal = autoware::universe_utils::calcDistance2d(next_pose, goal_pose_);
      next_node->dist_to_obs = distance_to_obs;
      next_node->parent = &current_node;
      openlist_.push(next_node);
      continue;
    }
  }
}

double AstarSearch::getExpansionDistance(const AstarNode & current_node) const
{
  if (!astar_param_.adapt_expansion_distance) return min_expansion_dist_;
  double exp_dist = std::min(
    current_node.dist_to_goal * dist_to_goal_expansion_factor_,
    current_node.dist_to_obs * dist_to_obs_expansion_factor_);
  return std::clamp(exp_dist, min_expansion_dist_, max_expansion_dist_);
}

double AstarSearch::getSteeringCost(const int steering_index) const
{
  return planner_common_param_.curve_weight *
         (abs(steering_index) / planner_common_param_.turning_steps);
}

double AstarSearch::getSteeringChangeCost(
  const int steering_index, const int prev_steering_index) const
{
  double steering_index_diff = abs(steering_index - prev_steering_index);
  return astar_param_.steering_change_weight * steering_index_diff /
         (2.0 * planner_common_param_.turning_steps);
}

void AstarSearch::setPath(const AstarNode & goal_node)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  header.frame_id = costmap_.header.frame_id;

  // From the goal node to the start node
  const AstarNode * node = &goal_node;

  std::vector<PlannerWaypoint> waypoints;
  // push astar nodes poses
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  while (node != nullptr) {
    pose.pose = local2global(costmap_, node2pose(*node));
    waypoints.push_back({pose, node->is_back});
    // To the next node
    node = node->parent;
  }

  if (waypoints.size() > 1) waypoints.back().is_back = waypoints.rbegin()[1].is_back;

  if (search_method_ != SearchMethod::Backward) {
    // Reverse the vector to be start to goal order
    std::reverse(waypoints.begin(), waypoints.end());
  }

  waypoints_.header = header;
  waypoints_.waypoints.clear();
  auto it = waypoints.begin();
  for (; it < waypoints.end() - 1; ++it) {
    auto next_it = it + 1;
    waypoints_.waypoints.push_back(*it);
    if (it->is_back == next_it->is_back) continue;
    if (search_method_ == SearchMethod::Backward) {
      waypoints_.waypoints.push_back({next_it->pose, it->is_back});
    } else {
      waypoints_.waypoints.push_back({it->pose, next_it->is_back});
    }
  }
}

bool AstarSearch::isGoal(const AstarNode & node) const
{
  const double lateral_goal_range = planner_common_param_.lateral_goal_range / 2.0;
  const double longitudinal_goal_range = planner_common_param_.longitudinal_goal_range / 2.0;
  const double goal_angle =
    autoware::universe_utils::deg2rad(planner_common_param_.angle_goal_range / 2.0);

  const auto relative_pose = calcRelativePose(goal_pose_, node2pose(node));

  // Check conditions
  if (astar_param_.only_behind_solutions && relative_pose.position.x > 0) {
    return false;
  }

  if (
    std::fabs(relative_pose.position.x) > longitudinal_goal_range ||
    std::fabs(relative_pose.position.y) > lateral_goal_range) {
    return false;
  }

  const auto angle_diff =
    autoware::universe_utils::normalizeRadian(tf2::getYaw(relative_pose.orientation));
  if (std::abs(angle_diff) > goal_angle) {
    return false;
  }

  return true;
}

geometry_msgs::msg::Pose AstarSearch::node2pose(const AstarNode & node) const
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = goal_pose_.position.z;
  pose_local.orientation = autoware::universe_utils::createQuaternionFromYaw(node.theta);

  return pose_local;
}

}  // namespace autoware::freespace_planning_algorithms
