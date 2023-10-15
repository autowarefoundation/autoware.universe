// Copyright 2023 Tier IV, Inc.
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

#include "mrm_pull_over_manager/mrm_pull_over_manager_core.hpp"

#include "mrm_pull_over_manager/csv_parser.hpp"
namespace mrm_pull_over_manager
{

// TODO: temporary
namespace
{
// geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
// {
//   geometry_msgs::msg::Quaternion ros_quat;

//   // Convert yaw angle (theta) to quaternion
//   ros_quat.x = 0.0;
//   ros_quat.y = 0.0;
//   ros_quat.z = sin(yaw * 0.5);
//   ros_quat.w = cos(yaw * 0.5);

//   return ros_quat;
// }

// geometry_msgs::msg::Pose create_pose(
//   const double x, const double y, const double z, const double yaw)
// {
//   geometry_msgs::msg::Pose pose;
//   pose.position.x = x;
//   pose.position.y = y;
//   pose.position.z = z;
//   pose.orientation = yaw_to_quaternion(yaw);

//   return pose;
// }
}  // namespace

namespace lanelet_util
{
/**
 * @brief Retrieve the current root lanelet based on the provided pose.
 * @param route_handler Reference to a route handler that provides lanelet related utilities.
 * @param current_pose Current pose used to determine the relevant lanelet.
 * @return The lanelet that corresponds to the current pose.
 */
lanelet::ConstLanelet get_current_lanelet(
  const route_handler::RouteHandler & route_handler, const geometry_msgs::msg::Pose & current_pose)
{
  lanelet::ConstLanelet current_lanelet;
  route_handler.getClosestLaneletWithinRoute(current_pose, &current_lanelet);

  return current_lanelet;
}

/**
 * @brief Retrieve all left adjacent lanelets relative to the base lanelet.
 * @param route_handler Reference to a route handler that provides lanelet related utilities.
 * @param base_lanelet The reference lanelet from which left adjacent lanelets are determined.
 * @return A collection of lanelets that are to the left of the base lanelet.
 */
lanelet::ConstLanelets get_all_left_lanelets(
  const route_handler::RouteHandler & route_handler, const lanelet::ConstLanelet & base_lanelet)
{
  lanelet::ConstLanelets left_lanalets;
  auto left_lanelet = route_handler.getLeftLanelet(base_lanelet, false, true);
  while (left_lanelet) {
    left_lanalets.emplace_back(left_lanelet.get());
    RCLCPP_INFO(rclcpp::get_logger(__func__), "left lanelet id: %ld", left_lanelet->id());

    // get next left lanelet
    left_lanelet = route_handler.getLeftLanelet(left_lanelet.get(), false, true);
  }

  return left_lanalets;
}

/**
 * @brief Retrieve all following lanes and left adjacent lanes starting from the provided start
 * lanelet.
 * @param route_handler Reference to a route handler that provides lanelet related utilities.
 * @param start_lanelet The initial lanelet from which following are determined.
 * @return A collection of lanelets that are following and to the left of the start lanelet.
 */
lanelet::ConstLanelets get_all_following_and_left_lanelets(
  const route_handler::RouteHandler & route_handler, const lanelet::ConstLanelet & start_lanelet)
{
  lanelet::ConstLanelet current_lanelet = start_lanelet;
  lanelet::ConstLanelets result_lanelets;
  result_lanelets.emplace_back(start_lanelet);

  // Update current_lanelet to the next following lanelet, if any
  while (route_handler.getNextLaneletWithinRoute(current_lanelet, &current_lanelet)) {
    RCLCPP_INFO(rclcpp::get_logger(__func__), "current lanelet id: %ld", current_lanelet.id());

    result_lanelets.emplace_back(current_lanelet);

    // Add all left lanelets
    auto left_lanelets = get_all_left_lanelets(route_handler, current_lanelet);
    result_lanelets.insert(result_lanelets.end(), left_lanelets.begin(), left_lanelets.end());
  }

  return result_lanelets;
}
}  // namespace lanelet_util

MrmPullOverManager::MrmPullOverManager() : Node("mrm_pull_over_manager")
{
  // Parameter
  // param_.update_rate = declare_parameter<int>("update_rate", 10);

  using std::placeholders::_1;

  // Subscriber
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&MrmPullOverManager::on_odometry, this, _1));
  sub_route_ = this->create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&MrmPullOverManager::on_route, this, _1));
  sub_map_ = create_subscription<HADMapBin>(
    "~/input/lanelet_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MrmPullOverManager::on_map, this, _1));
  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1}, std::bind(&MrmPullOverManager::on_trajectory, this, _1));

  // Publisher
  pub_pose_array_ = create_publisher<PoseArray>("~/output/emergency_goals", rclcpp::QoS{1});

  const std::string filepath = "/media/tomohitoando/data/map/shiojiri_100laps/pull_over_point.csv";
  CsvPoseParser parser(filepath);
  candidate_goals_ = parser.parse();

  // TODO: temporary
  // Timer
  const auto update_period_ns = rclcpp::Rate(10.0).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmPullOverManager::on_timer, this));
}

void MrmPullOverManager::on_timer()
{
  const bool result = find_goals_within_route();
  RCLCPP_INFO_STREAM(this->get_logger(), "result: " << result);
}

void MrmPullOverManager::activatePullOver(
  const tier4_system_msgs::srv::ActivatePullOver::Request::SharedPtr request,
  const tier4_system_msgs::srv::ActivatePullOver::Response::SharedPtr response)
{
  if (request->activate == true) {
    const bool result = find_goals_within_route();
    response->status.success = result;
  }

  if (request->activate == false) {
    // TODO: Call ClearEmergency service
    response->status.success = true;
  }
}

void MrmPullOverManager::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void MrmPullOverManager::on_route(const LaneletRoute::ConstSharedPtr msg)
{
  route_handler_.setRoute(*msg);
}

void MrmPullOverManager::on_map(const HADMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
}

void MrmPullOverManager::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  trajectory_ = msg;
}

bool MrmPullOverManager::is_data_ready()
{
  if (!odom_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for odometry msg...");
    return false;
  }

  if (!route_handler_.isHandlerReady()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for route handler.");
    return false;
  }

  if (!trajectory_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for trajectory.");
    return false;
  }

  return true;
}

bool MrmPullOverManager::find_goals_within_route()
{
  if (!is_data_ready()) {
    return false;
  }

  const auto current_lanelet = lanelet_util::get_current_lanelet(route_handler_, odom_->pose.pose);
  RCLCPP_INFO(this->get_logger(), "current lanelet id:%ld", current_lanelet.id());

  const auto candidate_lanelets =
    lanelet_util::get_all_following_and_left_lanelets(route_handler_, current_lanelet);

  PoseArray emergency_goals;
  emergency_goals.header.frame_id = "map";
  emergency_goals.header.stamp = now();
  emergency_goals.poses = find_goals_in_lanelets(candidate_lanelets);

  // temporary
  for (const auto & goal : emergency_goals.poses) {
    RCLCPP_INFO(this->get_logger(), "goal x: %f", goal.position.x);
  }

  emergency_goals.poses = filter_nearby_goals(emergency_goals.poses);

  pub_pose_array_->publish(emergency_goals);

  return true;
}

std::vector<geometry_msgs::msg::Pose> MrmPullOverManager::find_goals_in_lanelets(
  const lanelet::ConstLanelets & candidate_lanelets) const
{
  RCLCPP_INFO(this->get_logger(), "candidate lanelet count: %ld", candidate_lanelets.size());
  std::vector<Pose> goals;

  for (const auto & lane : candidate_lanelets) {
    const auto it = candidate_goals_.find(lane.id());

    // found the goal that has the same lanelet id
    if (it != candidate_goals_.end()) {
      goals.emplace_back(it->second);
    }
  }

  return goals;
}

std::vector<geometry_msgs::msg::Pose> MrmPullOverManager::filter_nearby_goals(
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  RCLCPP_INFO(this->get_logger(), "pose count: %ld", poses.size());
  std::vector<geometry_msgs::msg::Pose> filtered_poses;

  const double distance_threshold = 10.0;  // TODO
  const double yaw_threshold = 1.0;        // TODO
  auto it = poses.begin();
  for (; it != poses.end(); ++it) {
    // filter unsafe yaw pose
    const double yaw_deviation = motion_utils::calcYawDeviation(trajectory_->points, *it);
    RCLCPP_INFO(this->get_logger(), "yaw deviation to pose: %lf", yaw_deviation);
    if (std::abs(yaw_deviation) > yaw_threshold) {
      continue;
    }

    // filter too near pose
    // The poses are stored in order of distance, so the loop exits as soon as the first one
    // exceeding the threshold is found
    const double arc_length_to_pose = motion_utils::calcSignedArcLength(
      trajectory_->points, odom_->pose.pose.position, it->position);
    RCLCPP_INFO(this->get_logger(), "distance to the pose: %lf", arc_length_to_pose);
    if (arc_length_to_pose > distance_threshold) {
      break;
    }
  }

  // Store poses that have larger distance than threshold
  if (it != poses.end()) {
    filtered_poses.insert(filtered_poses.end(), it, poses.end());
  }

  return filtered_poses;
}
}  // namespace mrm_pull_over_manager
