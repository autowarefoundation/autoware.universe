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
    left_lanalets.emplace_back(left_lanelet.value());
    RCLCPP_DEBUG(rclcpp::get_logger(__func__), "left lanelet id: %ld", left_lanelet->id());

    // get next left lanelet
    left_lanelet = route_handler.getLeftLanelet(left_lanelet.value(), false, true);
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
  const route_handler::RouteHandler & route_handler,
  const autoware_planning_msgs::msg::LaneletRoute & route,
  const lanelet::ConstLanelet & start_lanelet)
{
  lanelet::ConstLanelets result_lanelets;
  bool found_start_lane = false;
  for (const auto & segment : route.segments) {
    // If the start lanelet hasn't been found, search for it
    if (!found_start_lane) {
      for (const auto & primitive : segment.primitives) {
        if (primitive.id == start_lanelet.id()) {
          found_start_lane = true;
          break;  // Break out once the start lanelet is found
        }
      }
    }

    // search lanelets from the start_lanelet
    if (!found_start_lane) {
      continue;
    }

    const auto current_lane = route_handler.getLaneletsFromId(segment.preferred_primitive.id);
    result_lanelets.emplace_back(current_lane);
    RCLCPP_DEBUG(rclcpp::get_logger(__func__), "current lanelet id: %ld", current_lane.id());

    // Add all left lanelets
    auto left_lanes = get_all_left_lanelets(route_handler, current_lane);
    result_lanelets.insert(result_lanelets.end(), left_lanes.begin(), left_lanes.end());
  }

  return result_lanelets;
}
}  // namespace lanelet_util

MrmPullOverManager::MrmPullOverManager() : Node("mrm_pull_over_manager")
{
  // Parameter
  param_.update_rate = declare_parameter<double>("update_rate");
  param_.pull_over_point_file_path = declare_parameter<std::string>("pull_over_point_file_path");
  param_.max_goal_pose_num = declare_parameter<int>("max_goal_pose_num");
  param_.yaw_deviation_threshold = declare_parameter<double>("yaw_deviation_threshold");
  param_.margin_time_to_goal = declare_parameter<double>("margin_time_to_goal");

  using std::placeholders::_1;
  using std::placeholders::_2;

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
  pub_emergency_goals_ = create_publisher<EmergencyGoalsStamped>(
    "~/output/mrm/pull_over/emergency_goals", rclcpp::QoS{1});
  pub_status_ =
    create_publisher<MrmBehaviorStatus>("~/output/mrm/pull_over/status", rclcpp::QoS{1});
  pub_emergency_goals_clear_command_ = create_publisher<EmergencyGoalsClearCommand>(
    "~/output/mrm/pull_over/goals_clear_command", rclcpp::QoS{1});

  // Server
  operate_mrm_ = create_service<tier4_system_msgs::srv::OperateMrm>(
    "~/input/mrm/pull_over/operate", std::bind(&MrmPullOverManager::operateMrm, this, _1, _2));

  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmPullOverManager::on_timer, this));

  // Parse CSV and store the emergency goals
  CsvPoseParser parser(param_.pull_over_point_file_path);
  candidate_goals_ = parser.parse();

  // Initialize the state
  status_.state = MrmBehaviorStatus::AVAILABLE;
}

std::string MrmPullOverManager::get_module_name() const
{
  return "mrm_pull_over_manager";
}

void MrmPullOverManager::on_timer()
{
  publish_status();
}

void MrmPullOverManager::publish_status() const
{
  auto status = status_;
  status.stamp = this->now();
  pub_status_->publish(status);
}

void MrmPullOverManager::publish_emergency_goals(const std::vector<Pose> & emergency_goals) const
{
  EmergencyGoalsStamped emergency_goals_stamped;
  emergency_goals_stamped.goals = emergency_goals;
  emergency_goals_stamped.stamp = now();
  emergency_goals_stamped.sender = get_module_name();
  pub_emergency_goals_->publish(emergency_goals_stamped);
}

void MrmPullOverManager::publish_emergency_goals_clear_command() const
{
  EmergencyGoalsClearCommand goals_clear_command;
  goals_clear_command.stamp = this->now();
  goals_clear_command.command = true;
  goals_clear_command.sender = get_module_name();

  pub_emergency_goals_clear_command_->publish(goals_clear_command);
}

void MrmPullOverManager::operateMrm(
  const tier4_system_msgs::srv::OperateMrm::Request::SharedPtr request,
  const tier4_system_msgs::srv::OperateMrm::Response::SharedPtr response)
{
  if (request->operate == true) {
    const bool result = find_goals_within_route();
    response->response.success = result;
    status_.state = MrmBehaviorStatus::OPERATING;
  }

  if (request->operate == false) {
    publish_emergency_goals_clear_command();
    response->response.success = true;
    status_.state = MrmBehaviorStatus::AVAILABLE;
  }
}

void MrmPullOverManager::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void MrmPullOverManager::on_route(const LaneletRoute::ConstSharedPtr msg)
{
  // TODO(TomohitoAndo): If we can get all the lanes including lane change, route_ is not required
  route_ = msg;
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
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for odometry msg...");
    return false;
  }

  if (!route_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for route...");
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
  RCLCPP_DEBUG(this->get_logger(), "current lanelet id:%ld", current_lanelet.id());

  const auto candidate_lanelets =
    lanelet_util::get_all_following_and_left_lanelets(route_handler_, *route_, current_lanelet);

  const auto emergency_goals = find_goals_in_lanelets(candidate_lanelets);

  const auto filtered_emergency_goals = filter_nearby_goals(emergency_goals);

  publish_emergency_goals(filtered_emergency_goals);

  return true;
}

std::vector<geometry_msgs::msg::Pose> MrmPullOverManager::find_goals_in_lanelets(
  const lanelet::ConstLanelets & candidate_lanelets) const
{
  RCLCPP_DEBUG(this->get_logger(), "candidate lanelet count: %ld", candidate_lanelets.size());
  std::vector<Pose> goals;

  for (const auto & lane : candidate_lanelets) {
    const auto it = candidate_goals_.find(lane.id());

    // found the goal that has the same lanelet id
    if (it != candidate_goals_.end()) {
      goals.emplace_back(it->second);
    }

    if (goals.size() >= param_.max_goal_pose_num) {
      break;
    }
  }

  return goals;
}

std::vector<geometry_msgs::msg::Pose> MrmPullOverManager::filter_nearby_goals(
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  RCLCPP_DEBUG(this->get_logger(), "pose count: %ld", poses.size());
  std::vector<geometry_msgs::msg::Pose> filtered_poses;

  auto it = poses.begin();
  for (; it != poses.end(); ++it) {
    // filter unsafe yaw pose
    const double yaw_deviation = motion_utils::calcYawDeviation(trajectory_->points, *it);
    RCLCPP_DEBUG(this->get_logger(), "yaw deviation to pose: %lf", yaw_deviation);
    if (std::abs(yaw_deviation) > param_.yaw_deviation_threshold) {
      continue;
    }

    // filter too near pose
    // The poses are stored in order of distance, so the loop exits as soon as the first one
    // exceeding the threshold is found
    // TODO(TomohitoAndo): Replace with better logic to filter the near goals
    const double arc_length_to_pose = motion_utils::calcSignedArcLength(
      trajectory_->points, odom_->pose.pose.position, it->position);
    const double distance_threshold = odom_->twist.twist.linear.x * param_.margin_time_to_goal;
    RCLCPP_DEBUG(this->get_logger(), "distance to the pose: %lf", arc_length_to_pose);
    RCLCPP_DEBUG(this->get_logger(), "distance threshold: %lf", distance_threshold);
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
