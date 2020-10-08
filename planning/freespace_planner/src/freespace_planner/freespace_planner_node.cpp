/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <freespace_planner/freespace_planner.h>

namespace
{
bool isActive(const autoware_planning_msgs::Scenario::ConstPtr & scenario)
{
  if (!scenario) {
    return false;
  }

  const auto & s = scenario->activating_scenarios;
  if (
    std::find(std::begin(s), std::end(s), autoware_planning_msgs::Scenario::Parking) !=
    std::end(s)) {
    return true;
  }

  return false;
}

geometry_msgs::PoseArray trajectory2posearray(const autoware_planning_msgs::Trajectory & trajectory)
{
  geometry_msgs::PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

std::vector<size_t> getReversingIndices(const autoware_planning_msgs::Trajectory & trajectory)
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

autoware_planning_msgs::Trajectory getPartialTrajectory(
  const autoware_planning_msgs::Trajectory & trajectory, const size_t start_index,
  const size_t end_index)
{
  autoware_planning_msgs::Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = ros::Time::now();

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

double calcDistance2d(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2)
{
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double calcDistance2d(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2)
{
  return calcDistance2d(p1.position, p2.position);
}

std::vector<double> calcDistances2d(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose)
{
  std::vector<double> distances;
  distances.reserve(trajectory.points.size());

  std::transform(
    std::begin(trajectory.points), std::end(trajectory.points), std::back_inserter(distances),
    [&](const auto & point) { return calcDistance2d(point.pose.position, pose.position); });

  return distances;
}

double calcDistance2d(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose)
{
  const auto distances = calcDistances2d(trajectory, pose);
  const auto min_itr = std::min_element(std::begin(distances), std::end(distances));
  return *min_itr;
}

geometry_msgs::Pose transformPose(
  const geometry_msgs::Pose & pose, const geometry_msgs::TransformStamped & transform)
{
  geometry_msgs::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

geometry_msgs::PoseStamped tf2pose(const geometry_msgs::TransformStamped & tf)
{
  geometry_msgs::PoseStamped pose;

  pose.header = tf.header;
  pose.pose.orientation = tf.transform.rotation;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;

  return pose;
}

autoware_planning_msgs::Trajectory createTrajectory(
  const geometry_msgs::PoseStamped & current_pose, const AstarWaypoints & astar_waypoints,
  const double & velocity)
{
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.header = astar_waypoints.header;

  for (const auto & awp : astar_waypoints.waypoints) {
    autoware_planning_msgs::TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.accel = {};
    point.twist = {};

    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.twist.linear.x = velocity / 3.6;                 // velocity = const

    // switch sign by forward/backward
    point.twist.linear.x = (awp.is_back ? -1 : 1) * point.twist.linear.x;

    trajectory.points.push_back(point);
  }

  return trajectory;
}

autoware_planning_msgs::Trajectory createStopTrajectory(
  const geometry_msgs::PoseStamped & current_pose)
{
  AstarWaypoints waypoints;
  AstarWaypoint waypoint;

  waypoints.header.stamp = ros::Time::now();
  waypoints.header.frame_id = current_pose.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose.pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);

  return createTrajectory(current_pose, waypoints, 0.0);
}

size_t findNearestIndex(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose)
{
  const auto distances = calcDistances2d(trajectory, pose);
  const auto min_itr = std::min_element(std::begin(distances), std::end(distances));
  return std::distance(std::begin(distances), min_itr);
}

bool isStopped(
  const std::deque<geometry_msgs::TwistStamped::ConstPtr> & twist_buffer,
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

AstarNavi::AstarNavi() : nh_(), private_nh_("~"), tf_listener_(tf_buffer_)
{
  // NodeParam
  {
    private_nh_.param<double>("waypoints_velocity", node_param_.waypoints_velocity, 5.0);
    private_nh_.param<double>("update_rate", node_param_.update_rate, 1.0);
    private_nh_.param<double>("th_arrived_distance_m", node_param_.th_arrived_distance_m, 1.0);
    private_nh_.param<double>("th_stopped_time_sec", node_param_.th_stopped_time_sec, 1.0);
    private_nh_.param<double>("th_stopped_velocity_mps", node_param_.th_stopped_velocity_mps, 0.01);
    private_nh_.param<double>(
      "th_course_out_distance_m", node_param_.th_course_out_distance_m, 3.0);
    private_nh_.param<bool>(
      "replan_when_obstacle_found", node_param_.replan_when_obstacle_found, true);
    private_nh_.param<bool>("replan_when_course_out", node_param_.replan_when_course_out, true);
  }

  // AstarParam
  {
    // base configs
    private_nh_.param<bool>("use_back", astar_param_.use_back, true);
    private_nh_.param<bool>("only_behind_solutions", astar_param_.only_behind_solutions, false);
    private_nh_.param<double>("time_limit", astar_param_.time_limit, 5000.0);

    // robot configs
    // TODO(Kenji Miyake): obtain from vehicle_info
    private_nh_.param<double>("robot_length", astar_param_.robot_shape.length, 4.5);
    private_nh_.param<double>("robot_width", astar_param_.robot_shape.width, 1.75);
    private_nh_.param<double>("robot_base2back", astar_param_.robot_shape.base2back, 1.0);
    private_nh_.param<double>("minimum_turning_radius", astar_param_.minimum_turning_radius, 6.0);

    // search configs
    private_nh_.param<int>("theta_size", astar_param_.theta_size, 48);
    private_nh_.param<double>("angle_goal_range", astar_param_.angle_goal_range, 6.0);
    private_nh_.param<double>("curve_weight", astar_param_.curve_weight, 1.2);
    private_nh_.param<double>("reverse_weight", astar_param_.reverse_weight, 2.00);
    private_nh_.param<double>("lateral_goal_range", astar_param_.lateral_goal_range, 0.5);
    private_nh_.param<double>("longitudinal_goal_range", astar_param_.longitudinal_goal_range, 2.0);

    // costmap configs
    private_nh_.param<int>("obstacle_threshold", astar_param_.obstacle_threshold, 100);
    private_nh_.param<double>(
      "distance_heuristic_weight", astar_param_.distance_heuristic_weight, 1.0);
  }

  // Subscribers
  {
    route_sub_ = private_nh_.subscribe("input/route", 1, &AstarNavi::onRoute, this);
    occupancy_grid_sub_ =
      private_nh_.subscribe("input/occupancy_grid", 1, &AstarNavi::onOccupancyGrid, this);
    scenario_sub_ = private_nh_.subscribe("input/scenario", 1, &AstarNavi::onScenario, this);
    twist_sub_ = private_nh_.subscribe("input/twist", 100, &AstarNavi::onTwist, this);
  }

  // Publishers
  {
    trajectory_pub_ =
      private_nh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1, true);

    debug_pose_array_pub_ =
      private_nh_.advertise<geometry_msgs::PoseArray>("debug/pose_array", 1, true);

    debug_partial_pose_array_pub_ =
      private_nh_.advertise<geometry_msgs::PoseArray>("debug/partial_pose_array", 1, true);
  }

  timer_ = private_nh_.createTimer(ros::Rate(node_param_.update_rate), &AstarNavi::onTimer, this);
}

void AstarNavi::onRoute(const autoware_planning_msgs::Route::ConstPtr & msg)
{
  route_ = msg;

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  reset();
}

void AstarNavi::onOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  occupancy_grid_ = msg;
}

void AstarNavi::onScenario(const autoware_planning_msgs::Scenario::ConstPtr & msg)
{
  scenario_ = msg;
}

void AstarNavi::onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  twist_ = msg;

  twist_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = msg->header.stamp - twist_buffer_.front()->header.stamp;

    if (time_diff.toSec() < node_param_.th_stopped_time_sec) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

bool AstarNavi::isPlanRequired()
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
      astar_->hasObstacleOnTrajectory(trajectory2posearray(forward_trajectory));
    if (is_obstacle_found) {
      ROS_INFO("Found obstacle");
      return true;
    }
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out =
      calcDistance2d(trajectory_, current_pose_.pose) > node_param_.th_course_out_distance_m;
    if (is_course_out) {
      ROS_INFO("Course out");
      return true;
    }
  }

  return false;
}

void AstarNavi::updateTargetIndex()
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
      private_nh_.setParam("is_completed", true);
      ROS_INFO_THROTTLE(1, "Astar completed");
    } else {
      // Switch to next partial trajectory
      prev_target_index_ = target_index_;
      target_index_ =
        getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);
    }
  }
}

void AstarNavi::onTimer(const ros::TimerEvent & event)
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

    // Stop before planning new trajectoryg
    const auto stop_trajectory = createStopTrajectory(current_pose_);
    trajectory_pub_.publish(stop_trajectory);
    debug_pose_array_pub_.publish(trajectory2posearray(stop_trajectory));
    debug_partial_pose_array_pub_.publish(trajectory2posearray(stop_trajectory));

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
  trajectory_pub_.publish(partial_trajectory_);
  debug_pose_array_pub_.publish(trajectory2posearray(trajectory_));
  debug_partial_pose_array_pub_.publish(trajectory2posearray(partial_trajectory_));
}

void AstarNavi::planTrajectory()
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
  const ros::WallTime start = ros::WallTime::now();
  const bool result = astar_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const ros::WallTime end = ros::WallTime::now();

  ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

  if (result) {
    ROS_INFO("Found goal!");
    trajectory_ =
      createTrajectory(current_pose_, astar_->getWaypoints(), node_param_.waypoints_velocity);
    reversing_indices_ = getReversingIndices(trajectory_);
    prev_target_index_ = 0;
    target_index_ =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);

  } else {
    ROS_INFO("Can't find goal...");
    reset();
  }
}

void AstarNavi::reset()
{
  trajectory_ = {};
  partial_trajectory_ = {};
  is_completed_ = false;
  private_nh_.setParam("is_completed", false);
}

geometry_msgs::TransformStamped AstarNavi::getTransform(
  const std::string & from, const std::string & to)
{
  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(from, to, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
  return tf;
}
