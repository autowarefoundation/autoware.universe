// Copyright 2022 TIER IV, Inc.
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


#ifndef MISSION_REMAINING_DISTANCE_TIME_CALCULATOR__HPP_
#define MISSION_REMAINING_DISTANCE_TIME_CALCULATOR__HPP_

//#include "mission_remaining_distance_time_calculator/mission_remaining_distance_time_calculator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <route_handler/route_handler.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_internal_msgs/msg/mission_remaining_distance_time.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>
#include <lanelet2_routing/LaneletPath.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>

namespace mission_remaining_distance_time_calculator
{

  using geometry_msgs::msg::Pose;
  using nav_msgs::msg::Odometry;
  using geometry_msgs::msg::Vector3;
  using autoware_planning_msgs::msg::LaneletRoute;
  using autoware_auto_mapping_msgs::msg::HADMapBin;
  using autoware_internal_msgs::msg::MissionRemainingDistanceTime;

struct NodeParam
{
  double update_rate{0.0};
};

class MissionRemainingDistanceTimeCalculatorNode : public rclcpp::Node
{
public:
  explicit MissionRemainingDistanceTimeCalculatorNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber
    rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscriber_;
  rclcpp::Subscription<HADMapBin>::SharedPtr map_subscriber_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscriber_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  // tier4_autoware_utils::SelfPoseListener self_pose_listener_;
  // rclcpp::Subscription<autoware_auto_planning_msgs::msg::Route>::SharedPtr sub_route_;

  // // Data Buffer
  // geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  // autoware_auto_planning_msgs::msg::Route::SharedPtr route_;

  // // Callback
  // void onRoute(const autoware_auto_planning_msgs::msg::Route::ConstSharedPtr & msg);

  // // Publisher
    rclcpp::Publisher<MissionRemainingDistanceTime>::SharedPtr
    mission_remaining_distance_time_publisher_;
  // tier4_autoware_utils::DebugPublisher debug_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  route_handler::RouteHandler route_handler_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  bool is_graph_ready_;

  void onTimer();
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr msg);
  double calcuateMissionRemainingDistance() const;
  double calcuateMissionRemainingTime(const double remaining_distance) const;
  void publishMissionRemainingDistanceTime(const double remaining_distance, const double remaining_time) const;

  Pose current_vehicle_pose_;
  Vector3 current_vehicle_velocity_;
  Pose goal_pose_;
  bool has_received_route_;

  // Parameter
  NodeParam node_param_;


  // Core
  // Input input_;
  // Output output_;
  // std::unique_ptr<GoalDistanceCalculator> goal_distance_calculator_;
};
}  // namespace mission_remaining_distance_time_calculator
#endif  // MISSION_REMAINING_DISTANCE_TIME_CALCULATOR__HPP_
