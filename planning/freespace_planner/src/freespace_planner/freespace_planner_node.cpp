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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autoware_utils/autoware_utils.hpp"
#include "freespace_planner/freespace_planner_node.hpp"

namespace
{
using autoware_planning_msgs::msg::Scenario;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::PlannerWaypoint;
using freespace_planning_algorithms::PlannerWaypoints;
using geometry_msgs::msg::Accel;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;

bool isActive(const Scenario::ConstSharedPtr & scenario)
{
  if (!scenario) {
    return false;
  }

  const auto & s = scenario->activating_scenarios;
  if (std::find(std::begin(s), std::end(s), Scenario::PARKING) != std::end(s)) {
    return true;
  }

  return false;
}

PoseArray trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

std::vector<size_t> getReversingIndices(const Trajectory & trajectory)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    if (trajectory.points.at(i).twist.linear.x * trajectory.points.at(i + 1).twist.linear.x < 0) {
      indices.push_back(i);
    }
  }

  return indices;
}

size_t getNextTargetIndex(
  const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
  const size_t current_target_index)
{
  if (!reversing_indices.empty()) {
    for (const auto reversing_index : reversing_indices) {
      if (reversing_index > current_target_index) {
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;
}

Trajectory getPartialTrajectory(
  const Trajectory & trajectory, const size_t start_index, const size_t end_index)
{
  Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = rclcpp::Clock().now();

  partial_trajectory.points.reserve(trajectory.points.size());
  for (size_t i = start_index; i <= end_index; ++i) {
    partial_trajectory.points.push_back(trajectory.points.at(i));
  }

  // Modify velocity at start/end point
  if (partial_trajectory.points.size() >= 2) {
    partial_trajectory.points.front().twist.linear.x =
      partial_trajectory.points.at(1).twist.linear.x;
  }
  if (!partial_trajectory.points.empty()) {
    partial_trajectory.points.back().twist.linear.x = 0;
  }

  return partial_trajectory;
}

double calcDistance2d(
  const Trajectory & trajectory, const Pose & pose)
{
  const auto idx = autoware_utils::findNearestIndex(trajectory.points, pose.position);
  return autoware_utils::calcDistance2d(trajectory.points.at(idx), pose);
}

Pose transformPose(
  const Pose & pose, const TransformStamped & transform)
{
  PoseStamped transformed_pose;
  PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

Trajectory createTrajectory(
  const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  Trajectory trajectory;
  trajectory.header = planner_waypoints.header;

  for (const auto & awp : planner_waypoints.waypoints) {
    TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.accel = Accel();
    point.twist = Twist();

    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.twist.linear.x = velocity / 3.6;                 // velocity = const

    // switch sign by forward/backward
    point.twist.linear.x = (awp.is_back ? -1 : 1) * point.twist.linear.x;

    trajectory.points.push_back(point);
  }

  return trajectory;
}

Trajectory createStopTrajectory(const PoseStamped & current_pose)
{
  PlannerWaypoints waypoints;
  PlannerWaypoint waypoint;

  waypoints.header.stamp = rclcpp::Clock().now();
  waypoints.header.frame_id = current_pose.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose.pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);

  return createTrajectory(current_pose, waypoints, 0.0);
}

bool isStopped(
  const std::deque<TwistStamped::ConstSharedPtr> & twist_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & twist : twist_buffer) {
    if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace freespace_planner
{
FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter("planning_algorithm", "astar");
    p.waypoints_velocity = declare_parameter("waypoints_velocity", 5.0);
    p.update_rate = declare_parameter("update_rate", 1.0);
    p.th_arrived_distance_m = declare_parameter("th_arrived_distance_m", 1.0);
    p.th_stopped_time_sec = declare_parameter("th_stopped_time_sec", 1.0);
    p.th_stopped_velocity_mps = declare_parameter("th_stopped_velocity_mps", 0.01);
    p.th_course_out_distance_m = declare_parameter("th_course_out_distance_m", 3.0);
    p.replan_when_obstacle_found = declare_parameter("replan_when_obstacle_found", true);
    p.replan_when_course_out = declare_parameter("replan_when_course_out", true);
    declare_parameter("is_completed");
  }

  // Planning
  getPlanningCommonParam();
  getAstarParam();

  // Subscribers
  {
    route_sub_ = create_subscription<Route>(
      "~/input/route", 1, std::bind(&FreespacePlannerNode::onRoute, this, _1));
    occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
      "~/input/occupancy_grid", 1, std::bind(&FreespacePlannerNode::onOccupancyGrid, this, _1));
    scenario_sub_ = create_subscription<Scenario>(
      "~/input/scenario", 1, std::bind(&FreespacePlannerNode::onScenario, this, _1));
    twist_sub_ = create_subscription<TwistStamped>(
      "~/input/twist", 100, std::bind(&FreespacePlannerNode::onTwist, this, _1));
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
    debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos);
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Timer
  {
    auto timer_callback = std::bind(&FreespacePlannerNode::onTimer, this);
    const auto period = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      get_clock(), period, std::move(timer_callback), get_node_base_interface()->get_context());
    get_node_timers_interface()->add_timer(timer_, nullptr);
  }
}

void FreespacePlannerNode::getPlanningCommonParam()
{
  auto & p = planner_common_param_;

  // base configs
  p.time_limit = declare_parameter("time_limit", 5000.0);

  // robot configs
  // TODO(Kenji Miyake): obtain from vehicle_info
  p.vehicle_shape.length = declare_parameter("robot_length", 4.5);
  p.vehicle_shape.width = declare_parameter("robot_width", 1.75);
  p.vehicle_shape.base2back = declare_parameter("robot_base2back", 1.0);
  p.minimum_turning_radius = declare_parameter("minimum_turning_radius", 0.5);
  p.maximum_turning_radius = declare_parameter("maximum_turning_radius", 6.0);
  p.turning_radius_size = declare_parameter("turning_radius_size", 11);
  p.maximum_turning_radius = std::max(
    p.maximum_turning_radius,
    p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  // search configs
  p.theta_size = declare_parameter("theta_size", 48);
  p.angle_goal_range = declare_parameter("angle_goal_range", 6.0);
  p.curve_weight = declare_parameter("curve_weight", 1.2);
  p.reverse_weight = declare_parameter("reverse_weight", 2.00);
  p.lateral_goal_range = declare_parameter("lateral_goal_range", 0.5);
  p.longitudinal_goal_range =
    declare_parameter("longitudinal_goal_range", 2.0);

  // costmap configs
  p.obstacle_threshold = declare_parameter("obstacle_threshold", 100);
}

void FreespacePlannerNode::getAstarParam()
{
  auto & p = astar_param_;
  p.only_behind_solutions = declare_parameter("astar.only_behind_solutions", false);
  p.use_back = declare_parameter("astar.use_back", true);
  p.distance_heuristic_weight = declare_parameter("astar.distance_heuristic_weight", 1.0);
}

void FreespacePlannerNode::onRoute(const Route::ConstSharedPtr msg)
{
  route_ = msg;

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  reset();
}

void FreespacePlannerNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

void FreespacePlannerNode::onScenario(
  const Scenario::ConstSharedPtr msg)
{
  scenario_ = msg;
}

void FreespacePlannerNode::onTwist(const TwistStamped::ConstSharedPtr msg)
{
  twist_ = msg;

  twist_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(twist_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found) {
    algo_->setMap(*occupancy_grid_);

    const size_t nearest_index_partial =
      autoware_utils::findNearestIndex(partial_trajectory_.points, current_pose_.pose.position);
    const size_t end_index_partial = partial_trajectory_.points.size() - 1;

    const auto forward_trajectory =
      getPartialTrajectory(partial_trajectory_, nearest_index_partial, end_index_partial);

    const bool is_obstacle_found =
      algo_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));
    if (is_obstacle_found) {
      RCLCPP_INFO(get_logger(), "Found obstacle");
      return true;
    }
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out =
      calcDistance2d(trajectory_, current_pose_.pose) > node_param_.th_course_out_distance_m;
    if (is_course_out) {
      RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }

  return false;
}

void FreespacePlannerNode::updateTargetIndex()
{
  const auto is_near_target =
    autoware_utils::calcDistance2d(trajectory_.points.at(target_index_), current_pose_) <
    node_param_.th_arrived_distance_m;

  const auto is_stopped = isStopped(twist_buffer_, node_param_.th_stopped_velocity_mps);

  if (is_near_target && is_stopped) {
    const auto new_target_index =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);

    if (new_target_index == target_index_) {
      // Finished publishing all partial trajectories
      is_completed_ = true;
      this->set_parameter(rclcpp::Parameter("is_completed", true));
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "Freespace planning completed");
    } else {
      // Switch to next partial trajectory
      prev_target_index_ = target_index_;
      target_index_ =
        getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);
    }
  }
}

void FreespacePlannerNode::onTimer()
{
  // Check all inputs are ready
  if (!occupancy_grid_ || !route_ || !scenario_ || !twist_) {
    return;
  }

  if (!isActive(scenario_)) {
    reset();
    return;
  }

  if (is_completed_) {
    return;
  }

  // Get current pose
  constexpr const char * vehicle_frame = "base_link";
  current_pose_ =
    autoware_utils::transform2pose(getTransform(occupancy_grid_->header.frame_id, vehicle_frame));
  if (current_pose_.header.frame_id == "") {
    return;
  }

  initializePlanningAlgorithm();
  if (isPlanRequired()) {
    reset();

    // Stop before planning new trajectory
    const auto stop_trajectory = createStopTrajectory(current_pose_);
    trajectory_pub_->publish(stop_trajectory);
    debug_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
    debug_partial_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));

    // Plan new trajectory
    planTrajectory();
  }

  // StopTrajectory
  if (trajectory_.points.size() <= 1) {
    return;
  }

  // Update partial trajectory
  updateTargetIndex();
  partial_trajectory_ = getPartialTrajectory(trajectory_, prev_target_index_, target_index_);

  // Publish messages
  trajectory_pub_->publish(partial_trajectory_);
  debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory_));
  debug_partial_pose_array_pub_->publish(trajectory2PoseArray(partial_trajectory_));
}

void FreespacePlannerNode::planTrajectory()
{
  // Extend robot shape
  freespace_planning_algorithms::VehicleShape extended_vehicle_shape =
    planner_common_param_.vehicle_shape;
  constexpr double margin = 1.0;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;

  // Provide robot shape and map for the planner
  algo_->setVehicleShape(extended_vehicle_shape);
  algo_->setMap(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = transformPose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = transformPose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute planning
  const rclcpp::Time start = get_clock()->now();
  const bool result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_INFO(get_logger(), "Found goal!");
    trajectory_ =
      createTrajectory(current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
    reversing_indices_ = getReversingIndices(trajectory_);
    prev_target_index_ = 0;
    target_index_ =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);

  } else {
    RCLCPP_INFO(get_logger(), "Can't find goal...");
    reset();
  }
}

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  this->set_parameter(rclcpp::Parameter("is_completed", false));
}

TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  if (node_param_.planning_algorithm == "astar") {
    algo_.reset(new AstarSearch(planner_common_param_, astar_param_));
  } else {
    throw std::runtime_error(
            "No such algorithm named " + node_param_.planning_algorithm + " exists.");
  }
}
}  // namespace freespace_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(freespace_planner::FreespacePlannerNode)
