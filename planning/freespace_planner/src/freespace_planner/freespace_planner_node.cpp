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

#include "freespace_planner/freespace_planner.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
bool isActive(const autoware_planning_msgs::msg::Scenario::ConstSharedPtr & scenario)
{
  if (!scenario) {
    return false;
  }

  const auto & s = scenario->activating_scenarios;
  if (
    std::find(std::begin(s), std::end(s), autoware_planning_msgs::msg::Scenario::PARKING) !=
    std::end(s))
  {
    return true;
  }

  return false;
}

geometry_msgs::msg::PoseArray trajectory2PoseArray(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

std::vector<size_t> getReversingIndices(const autoware_planning_msgs::msg::Trajectory & trajectory)
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

autoware_planning_msgs::msg::Trajectory getPartialTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const size_t start_index,
  const size_t end_index)
{
  autoware_planning_msgs::msg::Trajectory partial_trajectory;
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
  partial_trajectory.points.back().twist.linear.x = 0;

  return partial_trajectory;
}

double calcDistance2d(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double calcDistance2d(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  return calcDistance2d(p1.position, p2.position);
}

std::vector<double> calcDistances2d(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose)
{
  std::vector<double> distances;
  distances.reserve(trajectory.points.size());

  std::transform(
    std::begin(trajectory.points), std::end(trajectory.points), std::back_inserter(distances),
    [&](const auto & point) {return calcDistance2d(point.pose.position, pose.position);});

  return distances;
}

double calcDistance2d(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose)
{
  const auto distances = calcDistances2d(trajectory, pose);
  const auto min_itr = std::min_element(std::begin(distances), std::end(distances));
  return *min_itr;
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

geometry_msgs::msg::PoseStamped tf2pose(const geometry_msgs::msg::TransformStamped & tf)
{
  geometry_msgs::msg::PoseStamped pose;

  pose.header = tf.header;
  pose.pose.orientation = tf.transform.rotation;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;

  return pose;
}

autoware_planning_msgs::msg::Trajectory createTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose, const AstarWaypoints & astar_waypoints,
  const double & velocity)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header = astar_waypoints.header;

  for (const auto & awp : astar_waypoints.waypoints) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.accel = geometry_msgs::msg::Accel();
    point.twist = geometry_msgs::msg::Twist();

    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.twist.linear.x = velocity / 3.6;                 // velocity = const

    // switch sign by forward/backward
    point.twist.linear.x = (awp.is_back ? -1 : 1) * point.twist.linear.x;

    trajectory.points.push_back(point);
  }

  return trajectory;
}

autoware_planning_msgs::msg::Trajectory createStopTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose)
{
  AstarWaypoints waypoints;
  AstarWaypoint waypoint;

  waypoints.header.stamp = rclcpp::Clock().now();
  waypoints.header.frame_id = current_pose.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose.pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);

  return createTrajectory(current_pose, waypoints, 0.0);
}

size_t findNearestIndex(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose)
{
  const auto distances = calcDistances2d(trajectory, pose);
  const auto min_itr = std::min_element(std::begin(distances), std::end(distances));
  return std::distance(std::begin(distances), min_itr);
}

bool isStopped(
  const std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> & twist_buffer,
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

FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    node_param_.waypoints_velocity = declare_parameter("waypoints_velocity", 5.0);
    node_param_.update_rate = declare_parameter("update_rate", 1.0);
    node_param_.th_arrived_distance_m = declare_parameter("th_arrived_distance_m", 1.0);
    node_param_.th_stopped_time_sec = declare_parameter("th_stopped_time_sec", 1.0);
    node_param_.th_stopped_velocity_mps = declare_parameter("th_stopped_velocity_mps", 0.01);
    node_param_.th_course_out_distance_m = declare_parameter("th_course_out_distance_m", 3.0);
    node_param_.replan_when_obstacle_found = declare_parameter("replan_when_obstacle_found", true);
    node_param_.replan_when_course_out = declare_parameter("replan_when_course_out", true);
    declare_parameter<bool>("is_completed");
  }

  // AstarParam
  {
    // base configs
    astar_param_.use_back = declare_parameter("use_back", true);
    astar_param_.only_behind_solutions = declare_parameter("only_behind_solutions", false);
    astar_param_.time_limit = declare_parameter("time_limit", 5000.0);

    // robot configs
    // TODO(Kenji Miyake): obtain from vehicle_info
    astar_param_.robot_shape.length = declare_parameter("robot_length", 4.5);
    astar_param_.robot_shape.width = declare_parameter("robot_width", 1.75);
    astar_param_.robot_shape.base2back = declare_parameter("robot_base2back", 1.0);
    astar_param_.minimum_turning_radius = declare_parameter("minimum_turning_radius", 0.5);
    astar_param_.maximum_turning_radius = declare_parameter("maximum_turning_radius", 6.0);
    astar_param_.turning_radius_size = declare_parameter("turning_radius_size", 11);
    astar_param_.maximum_turning_radius = std::max(
      astar_param_.maximum_turning_radius,
      astar_param_.minimum_turning_radius);
    astar_param_.turning_radius_size = std::max(astar_param_.turning_radius_size, 1);

    // search configs
    astar_param_.theta_size = declare_parameter("theta_size", 48);
    astar_param_.angle_goal_range = declare_parameter("angle_goal_range", 6.0);
    astar_param_.curve_weight = declare_parameter("curve_weight", 1.2);
    astar_param_.reverse_weight = declare_parameter("reverse_weight", 2.00);
    astar_param_.lateral_goal_range = declare_parameter("lateral_goal_range", 0.5);
    astar_param_.longitudinal_goal_range = declare_parameter("longitudinal_goal_range", 2.0);

    // costmap configs
    astar_param_.obstacle_threshold = declare_parameter("obstacle_threshold", 100);
    astar_param_.distance_heuristic_weight = declare_parameter("distance_heuristic_weight", 1.0);
  }

  // Subscribers
  {
    route_sub_ = create_subscription<autoware_planning_msgs::msg::Route>(
      "~/input/route", rclcpp::QoS{1}, std::bind(&FreespacePlannerNode::onRoute, this, _1));
    occupancy_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "~/input/occupancy_grid", rclcpp::QoS{1},
      std::bind(&FreespacePlannerNode::onOccupancyGrid, this, _1));
    scenario_sub_ = create_subscription<autoware_planning_msgs::msg::Scenario>(
      "~/input/scenario", rclcpp::QoS{1}, std::bind(&FreespacePlannerNode::onScenario, this, _1));
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/input/twist", rclcpp::QoS{100}, std::bind(&FreespacePlannerNode::onTwist, this, _1));
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ =
      create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", qos);
    debug_pose_array_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("~/debug/pose_array", qos);
    debug_partial_pose_array_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("~/debug/partial_pose_array", qos);
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
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

void FreespacePlannerNode::onRoute(const autoware_planning_msgs::msg::Route::ConstSharedPtr msg)
{
  route_ = msg;

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  reset();
}

void FreespacePlannerNode::onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

void FreespacePlannerNode::onScenario(
  const autoware_planning_msgs::msg::Scenario::ConstSharedPtr msg)
{
  scenario_ = msg;
}

void FreespacePlannerNode::onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
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
    astar_.reset(new AstarSearch(astar_param_));
    astar_->initializeNodes(*occupancy_grid_);

    const auto nearest_index = findNearestIndex(trajectory_, current_pose_.pose);
    const auto forward_trajectory = getPartialTrajectory(trajectory_, nearest_index, target_index_);

    const bool is_obstacle_found =
      astar_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));
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
    calcDistance2d(trajectory_.points.at(target_index_).pose, current_pose_.pose) <
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
        get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "Astar completed");
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
  current_pose_ = tf2pose(getTransform(occupancy_grid_->header.frame_id, vehicle_frame));
  if (current_pose_.header.frame_id == "") {
    return;
  }

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
  RobotShape extended_robot_shape = astar_param_.robot_shape;
  constexpr double margin = 1.0;
  extended_robot_shape.length += margin;
  extended_robot_shape.width += margin;
  extended_robot_shape.base2back += margin / 2;

  // initialize vector for A* search, this runs only once
  astar_.reset(new AstarSearch(astar_param_));
  astar_->setRobotShape(extended_robot_shape);
  astar_->initializeNodes(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = transformPose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = transformPose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute astar search
  const rclcpp::Time start = get_clock()->now();
  const bool result = astar_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Astar planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_INFO(get_logger(), "Found goal!");
    trajectory_ =
      createTrajectory(current_pose_, astar_->getWaypoints(), node_param_.waypoints_velocity);
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
  trajectory_ = autoware_planning_msgs::msg::Trajectory();
  partial_trajectory_ = autoware_planning_msgs::msg::Trajectory();
  is_completed_ = false;
  this->set_parameter(rclcpp::Parameter("is_completed", false));
}

geometry_msgs::msg::TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(FreespacePlannerNode)
