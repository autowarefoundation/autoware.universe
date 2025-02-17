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

#include "autoware/scenario_selector/node.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>
namespace autoware::scenario_selector
{
namespace
{

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    current_position, lanelet_map_ptr, linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

bool isInLane(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Point & current_pos)
{
  const lanelet::BasicPoint2d search_point(current_pos.x, current_pos.y);

  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, 1);

  if (nearest_lanelets.empty()) return false;

  const auto dist_to_nearest_lanelet = nearest_lanelets.front().first;
  static constexpr double margin = 0.01;

  return dist_to_nearest_lanelet < margin;
}

bool isAlongLane(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::msg::Pose & current_pose)
{
  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(
        current_pose, &closest_lanelet, 0.0, M_PI_4)) {
    return false;
  }
  const lanelet::BasicPoint2d src_point(current_pose.position.x, current_pose.position.y);
  const auto dist_to_centerline =
    lanelet::geometry::distanceToCenterline2d(closest_lanelet, src_point);
  static constexpr double margin = 1.0;
  return dist_to_centerline < margin;
}

bool isInParkingLot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
    findNearestParkinglot(lanelet_map_ptr, search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

bool isNearTrajectoryEnd(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory,
  const geometry_msgs::msg::Pose & current_pose, const double th_dist)
{
  if (!trajectory || trajectory->points.empty()) {
    return false;
  }

  const auto & p1 = current_pose.position;
  const auto & p2 = trajectory->points.back().pose.position;

  const auto dist = std::hypot(p1.x - p2.x, p1.y - p2.y);

  return dist < th_dist;
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

autoware_planning_msgs::msg::Trajectory::ConstSharedPtr ScenarioSelectorNode::getScenarioTrajectory(
  const std::string & scenario)
{
  if (scenario == tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    return lane_driving_trajectory_;
  }
  if (scenario == tier4_planning_msgs::msg::Scenario::PARKING) {
    return parking_trajectory_;
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "invalid scenario argument: " << scenario);
  return lane_driving_trajectory_;
}

std::string ScenarioSelectorNode::selectScenarioByPosition()
{
  const auto is_in_lane =
    isInLane(route_handler_->getLaneletMapPtr(), current_pose_->pose.pose.position);
  const auto is_goal_in_lane =
    isInLane(route_handler_->getLaneletMapPtr(), route_->goal_pose.position);
  const auto is_in_parking_lot =
    isInParkingLot(route_handler_->getLaneletMapPtr(), current_pose_->pose.pose);

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::EMPTY) {
    if (is_in_lane && is_goal_in_lane) {
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    } else if (is_in_parking_lot) {
      return tier4_planning_msgs::msg::Scenario::PARKING;
    }
    return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
  }

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    if (is_in_parking_lot && !is_goal_in_lane) {
      return tier4_planning_msgs::msg::Scenario::PARKING;
    }
  }

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::PARKING) {
    if (is_parking_completed_ && is_in_lane) {
      is_parking_completed_ = false;
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    }
  }

  return current_scenario_;
}

void ScenarioSelectorNode::updateCurrentScenario()
{
  const auto prev_scenario = current_scenario_;

  const auto scenario_trajectory = getScenarioTrajectory(current_scenario_);
  const auto is_near_trajectory_end =
    isNearTrajectoryEnd(scenario_trajectory, current_pose_->pose.pose, th_arrived_distance_m_);

  const auto is_stopped = isStopped(twist_buffer_, th_stopped_velocity_mps_);

  if (is_near_trajectory_end && is_stopped) {
    current_scenario_ = selectScenarioByPosition();
  }

  if (enable_mode_switching_) {
    if (isCurrentLaneDriving()) {
      current_scenario_ = isSwitchToParking(is_stopped)
                            ? tier4_planning_msgs::msg::Scenario::PARKING
                            : current_scenario_;
    } else if (isCurrentParking()) {
      current_scenario_ = isSwitchToLaneDriving() ? tier4_planning_msgs::msg::Scenario::LANEDRIVING
                                                  : current_scenario_;
    }
  }

  if (current_scenario_ != prev_scenario) {
    lane_driving_stop_time_ = {};
    empty_parking_trajectory_time_ = {};
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "scenario changed: " << prev_scenario << " -> " << current_scenario_);
  }
}

bool ScenarioSelectorNode::isSwitchToParking(const bool is_stopped)
{
  const auto is_in_parking_lot =
    isInParkingLot(route_handler_->getLaneletMapPtr(), current_pose_->pose.pose);
  const auto is_goal_in_lane =
    isInLane(route_handler_->getLaneletMapPtr(), route_->goal_pose.position);

  if (!is_stopped || !isAutonomous() || !is_in_parking_lot || is_goal_in_lane) {
    lane_driving_stop_time_ = {};
    return false;
  }

  if (!lane_driving_stop_time_) {
    lane_driving_stop_time_ = this->now();
    return false;
  }

  return (this->now() - lane_driving_stop_time_.get()).seconds() > lane_stopping_timeout_s;
}

bool ScenarioSelectorNode::isSwitchToLaneDriving()
{
  const auto is_along_lane = isAlongLane(route_handler_, current_pose_->pose.pose);

  if (!isEmptyParkingTrajectory() || !is_along_lane) {
    empty_parking_trajectory_time_ = {};
    return false;
  }

  if (!empty_parking_trajectory_time_) {
    empty_parking_trajectory_time_ = this->now();
    return false;
  }

  const auto duration = (this->now() - empty_parking_trajectory_time_.get()).seconds();

  return duration > empty_parking_trajectory_timeout_s;
}

bool ScenarioSelectorNode::isAutonomous() const
{
  return operation_mode_state_->mode ==
           autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS &&
         operation_mode_state_->is_autoware_control_enabled;
}

bool ScenarioSelectorNode::isEmptyParkingTrajectory() const
{
  if (parking_trajectory_) {
    return parking_trajectory_->points.size() <= 1;
  }
  return false;
}

void ScenarioSelectorNode::onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  route_handler_ = std::make_shared<autoware::route_handler::RouteHandler>(*msg);
}

void ScenarioSelectorNode::onRoute(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  // When the route id is the same (e.g. rerouting with modified goal) keep the current scenario.
  // Otherwise, reset the scenario.
  if (!route_handler_ || route_handler_->getRouteUuid() != msg->uuid) {
    current_scenario_ = tier4_planning_msgs::msg::Scenario::EMPTY;
  }

  route_ = msg;
}

void ScenarioSelectorNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_pose_ = msg;
  auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist->header = msg->header;
  twist->twist = msg->twist.twist;

  twist_ = twist;
  twist_buffer_.push_back(twist);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(twist_buffer_.front()->header.stamp);

    if (time_diff.seconds() < th_stopped_time_sec_) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

bool ScenarioSelectorNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for current pose.");
    return false;
  }

  if (!route_handler_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for route handler.");
    return false;
  }

  if (!route_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for route.");
    return false;
  }

  if (!twist_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for twist.");
    return false;
  }

  if (!operation_mode_state_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for operation mode state.");
    return false;
  }

  // Check route handler is ready
  route_handler_->setRoute(*route_);
  if (!route_handler_->isHandlerReady()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for route handler.");
    return false;
  }

  return true;
}

void ScenarioSelectorNode::updateData()
{
  {
    stop_watch.tic();
  }
  {
    auto msg = sub_parking_state_->takeData();
    is_parking_completed_ = msg ? msg->data : is_parking_completed_;
  }

  {
    auto msgs = sub_odom_->takeData();
    for (const auto & msg : msgs) {
      onOdom(msg);
    }
  }

  {
    auto msg = sub_operation_mode_state_->takeData();
    if (msg) operation_mode_state_ = msg;
  }
}

void ScenarioSelectorNode::onTimer()
{
  updateData();

  if (!isDataReady()) {
    return;
  }

  // Initialize Scenario
  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::EMPTY) {
    current_scenario_ = selectScenarioByPosition();
  }

  updateCurrentScenario();
  tier4_planning_msgs::msg::Scenario scenario;
  scenario.current_scenario = current_scenario_;

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::PARKING) {
    scenario.activating_scenarios.push_back(current_scenario_);
  }

  pub_scenario_->publish(scenario);

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void ScenarioSelectorNode::onLaneDrivingTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  lane_driving_trajectory_ = msg;

  if (current_scenario_ != tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    return;
  }

  publishTrajectory(msg);
}

void ScenarioSelectorNode::onParkingTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  parking_trajectory_ = msg;

  if (current_scenario_ != tier4_planning_msgs::msg::Scenario::PARKING) {
    return;
  }

  publishTrajectory(msg);
}

void ScenarioSelectorNode::publishTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  const auto now = this->now();
  const auto delay_sec = (now - msg->header.stamp).seconds();
  if (delay_sec <= th_max_message_delay_sec_) {
    pub_trajectory_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(pub_trajectory_, msg->header.stamp);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "trajectory is delayed: scenario = %s, delay = %f, th_max_message_delay = %f",
      current_scenario_.c_str(), delay_sec, th_max_message_delay_sec_);
  }
}

ScenarioSelectorNode::ScenarioSelectorNode(const rclcpp::NodeOptions & node_options)
: Node("scenario_selector", node_options),
  current_scenario_(tier4_planning_msgs::msg::Scenario::EMPTY),
  update_rate_(this->declare_parameter<double>("update_rate")),
  th_max_message_delay_sec_(this->declare_parameter<double>("th_max_message_delay_sec")),
  th_arrived_distance_m_(this->declare_parameter<double>("th_arrived_distance_m")),
  th_stopped_time_sec_(this->declare_parameter<double>("th_stopped_time_sec")),
  th_stopped_velocity_mps_(this->declare_parameter<double>("th_stopped_velocity_mps")),
  enable_mode_switching_(this->declare_parameter<bool>("enable_mode_switching")),
  is_parking_completed_(false)
{
  lane_driving_stop_time_ = {};
  empty_parking_trajectory_time_ = {};

  // Input
  sub_lane_driving_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/lane_driving/trajectory", rclcpp::QoS{1},
    std::bind(&ScenarioSelectorNode::onLaneDrivingTrajectory, this, std::placeholders::_1));

  sub_parking_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/parking/trajectory", rclcpp::QoS{1},
    std::bind(&ScenarioSelectorNode::onParkingTrajectory, this, std::placeholders::_1));

  sub_lanelet_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/lanelet_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ScenarioSelectorNode::onMap, this, std::placeholders::_1));

  sub_route_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&ScenarioSelectorNode::onRoute, this, std::placeholders::_1));

  sub_odom_ = decltype(sub_odom_)::element_type::create_subscription(
    this, "input/odometry", rclcpp::QoS{100});

  sub_parking_state_ = decltype(sub_parking_state_)::element_type::create_subscription(
    this, "is_parking_completed", rclcpp::QoS{1});

  sub_operation_mode_state_ =
    decltype(sub_operation_mode_state_)::element_type::create_subscription(
      this, "input/operation_mode_state", rclcpp::QoS{1});

  // Output
  pub_scenario_ =
    this->create_publisher<tier4_planning_msgs::msg::Scenario>("output/scenario", rclcpp::QoS{1});
  pub_trajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "output/trajectory", rclcpp::QoS{1});

  // Timer Callback
  const auto period_ns = rclcpp::Rate(static_cast<double>(update_rate_)).period();

  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ScenarioSelectorNode::onTimer, this));
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
}
}  // namespace autoware::scenario_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::scenario_selector::ScenarioSelectorNode)
