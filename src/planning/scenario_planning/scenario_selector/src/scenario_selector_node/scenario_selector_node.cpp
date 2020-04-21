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
#include <scenario_selector/scenario_selector_node.h>

#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>

namespace
{
template <class T>
void onData(const T & data, T * buffer)
{
  *buffer = data;
}

template <class T>
boost::function<void(const T &)> createCallback(T * buffer)
{
  return static_cast<boost::function<void(const T &)>>(boost::bind(onData<T>, _1, buffer));
}

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & search_point)
{
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, 1);

  if (nearest_lanelets.empty()) {
    return {};
  }

  const auto nearest_lanelet = nearest_lanelets.front().second;
  const auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);

  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    nearest_lanelet, all_parking_lots, linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

geometry_msgs::PoseStamped::ConstPtr getCurrentPose(const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::TransformStamped tf_current_pose;

  try {
    tf_current_pose =
      tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("[scenario_selector] %s", ex.what());
    return nullptr;
  }

  geometry_msgs::PoseStamped::Ptr p(new geometry_msgs::PoseStamped());
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return geometry_msgs::PoseStamped::ConstPtr(p);
}

bool isInLane(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point.basicPoint2d(), 1);

  if (nearest_lanelets.empty()) {
    return false;
  }

  const auto nearest_lanelet = nearest_lanelets.front().second;

  return lanelet::geometry::within(search_point, nearest_lanelet.polygon3d());
}

bool isInParkingLot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::Pose & current_pose)
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
  const autoware_planning_msgs::Trajectory::ConstPtr & trajectory,
  const geometry_msgs::Pose & current_pose, const double th_dist)
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

Input ScenarioSelectorNode::getScenarioInput(const std::string & scenario)
{
  if (scenario == autoware_planning_msgs::Scenario::LaneDriving) return input_lane_driving_;
  if (scenario == autoware_planning_msgs::Scenario::Parking) return input_parking_;
  ROS_ERROR_STREAM("invalid scenario argument: " << scenario);
  return input_lane_driving_;
}

std::string ScenarioSelectorNode::selectScenarioByPosition()
{
  const auto is_in_lane = isInLane(lanelet_map_ptr_, current_pose_->pose);
  const auto is_goal_in_lane = isInLane(lanelet_map_ptr_, route_->goal_pose);
  const auto is_in_parking_lot = isInParkingLot(lanelet_map_ptr_, current_pose_->pose);

  if (current_scenario_ == autoware_planning_msgs::Scenario::Empty) {
    if (is_in_lane) {
      return autoware_planning_msgs::Scenario::LaneDriving;
    } else {
      return autoware_planning_msgs::Scenario::Parking;
    }
  }

  if (current_scenario_ == autoware_planning_msgs::Scenario::LaneDriving) {
    if (is_in_parking_lot && !is_goal_in_lane) {
      return autoware_planning_msgs::Scenario::Parking;
    }
  }

  if (current_scenario_ == autoware_planning_msgs::Scenario::Parking) {
    const auto is_parking_completed = nh_.param<bool>("is_parking_completed", false);
    if (is_parking_completed && is_in_lane) {
      nh_.setParam("is_parking_completed", false);
      return autoware_planning_msgs::Scenario::LaneDriving;
    }
  }

  return current_scenario_;
}

autoware_planning_msgs::Scenario ScenarioSelectorNode::selectScenario()
{
  const auto prev_scenario = current_scenario_;

  const auto scenario_trajectory = getScenarioInput(current_scenario_).buf_trajectory;

  const auto is_near_trajectory_end =
    isNearTrajectoryEnd(scenario_trajectory, current_pose_->pose, th_arrived_distance_m_);

  const auto is_stopped = isStopped(twist_buffer_, th_stopped_velocity_mps_);

  if (is_near_trajectory_end && is_stopped) {
    current_scenario_ = selectScenarioByPosition();
  }

  autoware_planning_msgs::Scenario scenario;
  scenario.current_scenario = current_scenario_;

  if (current_scenario_ == autoware_planning_msgs::Scenario::Parking) {
    scenario.activating_scenarios.push_back(current_scenario_);
  }

  if (current_scenario_ != prev_scenario) {
    ROS_INFO_STREAM("scenario changed: " << prev_scenario << " -> " << current_scenario_);
  }

  return scenario;
}

void ScenarioSelectorNode::onMap(const autoware_lanelet2_msgs::MapBin & msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void ScenarioSelectorNode::onRoute(const autoware_planning_msgs::Route::ConstPtr & msg)
{
  route_ = msg;
  current_scenario_ = autoware_planning_msgs::Scenario::Empty;
}

void ScenarioSelectorNode::onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  twist_ = msg;

  twist_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = msg->header.stamp - twist_buffer_.front()->header.stamp;

    if (time_diff.toSec() < th_stopped_time_sec_) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

void ScenarioSelectorNode::onTimer(const ros::TimerEvent & event)
{
  current_pose_ = getCurrentPose(tf_buffer_);

  // Check all inputs are ready
  if (!current_pose_ || !lanelet_map_ptr_ || !route_ || !twist_) {
    return;
  }

  // Initialize Scenario
  if (current_scenario_ == autoware_planning_msgs::Scenario::Empty) {
    current_scenario_ = selectScenarioByPosition();
  }

  // Select scenario
  const auto scenario = selectScenario();
  output_.pub_scenario.publish(scenario);

  const auto & input = getScenarioInput(scenario.current_scenario);

  if (!input.buf_trajectory) {
    return;
  }

  // Output
  const auto now = ros::Time::now();
  const auto delay_sec = (now - input.buf_trajectory->header.stamp).toSec();
  if (delay_sec <= th_max_message_delay_sec_) {
    output_.pub_trajectory.publish(input.buf_trajectory);
  } else {
    ROS_WARN_THROTTLE(
      1.0, "trajectory is delayed: scenario = %s, delay = %f, th_max_message_delay = %f",
      current_scenario_.c_str(), delay_sec, th_max_message_delay_sec_);
  }
}

ScenarioSelectorNode::ScenarioSelectorNode()
: nh_(""),
  private_nh_("~"),
  tf_listener_(tf_buffer_),
  current_scenario_(autoware_planning_msgs::Scenario::Empty)
{
  // Parameters
  private_nh_.param<double>("update_rate", update_rate_, 10.0);
  private_nh_.param<double>("th_max_message_delay_sec", th_max_message_delay_sec_, 1.0);
  private_nh_.param<double>("th_arrived_distance_m", th_arrived_distance_m_, 1.0);
  private_nh_.param<double>("th_stopped_time_sec", th_stopped_time_sec_, 1.0);
  private_nh_.param<double>("th_stopped_velocity_mps", th_stopped_velocity_mps_, 0.01);

  // Input
  input_lane_driving_.sub_trajectory = private_nh_.subscribe(
    "input/lane_driving/trajectory", 1, createCallback(&input_lane_driving_.buf_trajectory));

  input_parking_.sub_trajectory = private_nh_.subscribe(
    "input/parking/trajectory", 1, createCallback(&input_parking_.buf_trajectory));

  sub_lanelet_map_ =
    private_nh_.subscribe("input/lanelet_map", 1, &ScenarioSelectorNode::onMap, this);
  sub_route_ = private_nh_.subscribe("input/route", 1, &ScenarioSelectorNode::onRoute, this);
  sub_twist_ = private_nh_.subscribe("input/twist", 100, &ScenarioSelectorNode::onTwist, this);

  // Output
  output_.pub_scenario =
    private_nh_.advertise<autoware_planning_msgs::Scenario>("output/scenario", 1);
  output_.pub_trajectory =
    private_nh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);

  // Timer Callback
  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &ScenarioSelectorNode::onTimer, this);

  // Wait for first tf
  while (ros::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
      break;
    } catch (tf2::TransformException ex) {
      ROS_DEBUG("waiting for initial pose...");
    }
  }
}
