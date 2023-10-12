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

namespace mrm_pull_over_manager
{

namespace
{
PoseWithLaneId create_pose_with_lane_id(
  const double x, const double y, const double z, const lanelet::Id lanelet_id)
{
  PoseWithLaneId pose_with_lane_id;
  pose_with_lane_id.pose = geometry_msgs::msg::Pose();
  pose_with_lane_id.pose.position.x = x;
  pose_with_lane_id.pose.position.y = y;
  pose_with_lane_id.pose.position.z = z;
  pose_with_lane_id.lane_id = lanelet_id;

  return pose_with_lane_id;
}
}  // namespace

MrmPullOverManager::MrmPullOverManager() : Node("mrm_pull_over_manager")
{
  // Parameter
  // param_.update_rate = declare_parameter<int>("update_rate", 10);
  // param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);
  // param_.timeout_takeover_request = declare_parameter<double>("timeout_takeover_request", 10.0);
  // param_.use_takeover_request = declare_parameter<bool>("use_takeover_request", false);
  // param_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped", false);
  // param_.use_comfortable_stop = declare_parameter<bool>("use_comfortable_stop", false);
  // param_.turning_hazard_on.emergency = declare_parameter<bool>("turning_hazard_on.emergency",
  // true);

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

  // Publisher
  // pub_control_command_ =
  // create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
  //   "~/output/control_command", rclcpp::QoS{1});
  // pub_hazard_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
  //   "~/output/hazard", rclcpp::QoS{1});
  // pub_gear_cmd_ =
  //   create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("~/output/gear",
  //   rclcpp::QoS{1});
  // pub_mrm_state_ =
  //   create_publisher<autoware_adapi_v1_msgs::msg::MrmState>("~/output/mrm/state",
  //   rclcpp::QoS{1});

  // Clients
  // client_mrm_comfortable_stop_group_ =
  //   create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // client_mrm_comfortable_stop_ = create_client<tier4_system_msgs::srv::OperateMrm>(
  //   "~/output/mrm/comfortable_stop/operate", rmw_qos_profile_services_default,
  //   client_mrm_comfortable_stop_group_);

  // TODO: temporary
  // Timer
  const auto update_period_ns = rclcpp::Rate(10.0).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmPullOverManager::on_timer, this));
}

void MrmPullOverManager::on_timer()
{
  if (is_data_ready()) {
    const auto emergency_goals = find_near_goals();
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

  return true;
}

std::vector<geometry_msgs::msg::Pose> MrmPullOverManager::find_near_goals()
{
  const auto current_lanelet = get_current_lanelet();
  RCLCPP_INFO(this->get_logger(), "current lanelet id:%ld", current_lanelet.id());

  const auto candidate_lanelets = get_all_following_and_left_lanelets(current_lanelet);

  candidate_goals_.emplace_back(create_pose_with_lane_id(1, 1, 1, 19464));
  candidate_goals_.emplace_back(create_pose_with_lane_id(2, 2, 2, 19779));
  candidate_goals_.emplace_back(create_pose_with_lane_id(3, 3, 3, 20976));
  candidate_goals_.emplace_back(create_pose_with_lane_id(4, 4, 4, 20478));
  const auto emergency_goals = find_goals_in_lanelets(candidate_lanelets);

  return emergency_goals;
}

/**
 * @brief get current root lanelet
 * @param
 * @return current lanelet
 */
lanelet::ConstLanelet MrmPullOverManager::get_current_lanelet()
{
  lanelet::ConstLanelet current_lanelet;
  route_handler_.getClosestLaneletWithinRoute(odom_->pose.pose, &current_lanelet);

  return current_lanelet;
}

/**
 * @brief get all lanes followed by start_lanelet and left adjacent lanes.
 * @param start_lanelet
 * @return current lanelet
 */
lanelet::ConstLanelets MrmPullOverManager::get_all_following_and_left_lanelets(
  const lanelet::ConstLanelet & start_lanelet) const
{
  lanelet::ConstLanelet base_lanelet = start_lanelet;
  lanelet::ConstLanelet next_lanelet;
  lanelet::ConstLanelets result_lanelets;
  while (true) {
    RCLCPP_INFO(this->get_logger(), "base lanelet id: %ld", base_lanelet.id());
    result_lanelets.emplace_back(base_lanelet);

    // If there are multiple lanes to the left of the current lane, retrieve all of them
    const auto left_lanelets = get_all_left_lanelets(base_lanelet);
    result_lanelets.insert(result_lanelets.end(), left_lanelets.begin(), left_lanelets.end());

    // search next following lanelet
    const bool has_next = route_handler_.getNextLaneletWithinRoute(base_lanelet, &next_lanelet);
    if (!has_next) {
      break;
    }
    base_lanelet = next_lanelet;
  }

  return result_lanelets;
}

/**
 * @brief get all lanes followed by state_lanelet and left adjacent lanes of those lanes.
 * @param base_lanelet
 * @return
 */
lanelet::ConstLanelets MrmPullOverManager::get_all_left_lanelets(
  const lanelet::ConstLanelet & base_lanelet) const
{
  lanelet::ConstLanelets left_lanalets;
  auto left_lanelet = route_handler_.getLeftLanelet(base_lanelet, false, true);
  while (left_lanelet) {
    left_lanalets.emplace_back(left_lanelet.get());
    RCLCPP_INFO(this->get_logger(), "left lanelet id: %ld", left_lanelet->id());

    // get next left lanelet
    left_lanelet = route_handler_.getLeftLanelet(left_lanelet.get(), false, true);
  }

  return left_lanalets;
}

/**
 * @brief
 * @param candidate_lanelets
 * @return
 */
std::vector<geometry_msgs::msg::Pose> MrmPullOverManager::find_goals_in_lanelets(
  const lanelet::ConstLanelets & candidate_lanelets) const
{
  std::vector<geometry_msgs::msg::Pose> goals;
  return goals;
}

}  // namespace mrm_pull_over_manager
