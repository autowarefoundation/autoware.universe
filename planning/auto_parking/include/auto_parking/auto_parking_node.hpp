// Copyright 2024 TIER IV, Inc.
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
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef AUTO_PARKING__AUTO_PARKING_NODE_HPP_
#define AUTO_PARKING__AUTO_PARKING_NODE_HPP_

#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <freespace_planning_algorithms/astar_search.hpp>
#include <motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;

using EngageMsg = autoware_auto_vehicle_msgs::msg::Engage;
using EngageSrv = tier4_external_api_msgs::srv::Engage;

using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::AstarParam;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::VehicleShape;
using nav_msgs::msg::OccupancyGrid;

namespace auto_parking
{

struct AutoParkParam
{
  double th_arrived_distance_m;        // threshold to check arrival at goal 1.0
  double th_parking_space_distance_m;  // search for parking spaces within this radius 10
  double update_rate;                  // Timer() update rate 2
  double vehicle_shape_margin_m;  // margin to add to vehicle dimensions for astar collision check,
                                  // collision margin 0.2
};

class AutoParkingNode : public rclcpp::Node
{
public:
  explicit AutoParkingNode(const rclcpp::NodeOptions & node_options);

  void onMap(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void goalPublisher(const PoseStamped msg);
  void reset();
  void onTimer();
  void filterGoalPoseinParkingLot(const lanelet::ConstLineString3d center_line, Pose & goal);
  bool findParkingSpace();

  bool isInParkingLot();
  bool initAutoParking();
  bool isArrived(const Pose & goal);

  void onEngage(EngageMsg::ConstSharedPtr msg);
  void engageAutonomous();
  void onSetActiveStatus(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  // functions used in the fpa constructor
  PlannerCommonParam getPlannerCommonParam();
  TransformStamped getTransform(const std::string & from, const std::string & to);

private:
  // ros
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;

  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_status_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr lanelet_map_sub_;
  rclcpp::Subscription<EngageMsg>::SharedPtr engage_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  rclcpp::Client<EngageSrv>::SharedPtr client_engage_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_active_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::ConstPolygon3d> nearest_parking_lot_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::ConstLineStrings3d nearest_parking_spaces_;
  lanelet::ConstLanelets parking_lot_lanelets_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;

  // fpa algo vars
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;

  // params
  AutoParkParam node_param_;
  VehicleShape vehicle_shape_;

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

  PoseStamped current_goal_;
  PoseStamped current_pose_;
  lanelet::ConstLanelet current_lanelet_;
  std::unique_ptr<motion_utils::VehicleStopCheckerBase> stop_checker_;
  Pose parking_goal_;
  bool set_parking_lot_goal_;
  bool set_parking_space_goal_;
  bool active_;
  bool is_engaged_;
};
}  // namespace auto_parking
#endif  // AUTO_PARKING__AUTO_PARKING_NODE_HPP_
