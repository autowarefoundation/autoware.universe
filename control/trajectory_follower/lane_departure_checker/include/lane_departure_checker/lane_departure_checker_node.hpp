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

#ifndef LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_
#define LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_

#include <memory>
#include <vector>

#include "autoware_utils/ros/debug_publisher.hpp"
#include "autoware_utils/ros/processing_time_publisher.hpp"
#include "autoware_utils/ros/self_pose_listener.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_debug_msgs/msg/float64_stamped.hpp"
#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "lane_departure_checker/lane_departure_checker.hpp"

namespace lane_departure_checker
{
struct NodeParam
{
  double update_rate;
  bool visualize_lanelet;
};

class LaneDepartureCheckerNode : public rclcpp::Node
{
public:
  explicit LaneDepartureCheckerNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber
  autoware_utils::SelfPoseListener self_pose_listener_{this};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr sub_lanelet_map_bin_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr sub_route_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_reference_trajectory_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_predicted_trajectory_;

  // Data Buffer
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_twist_;
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  autoware_planning_msgs::msg::Route::ConstSharedPtr route_;
  autoware_planning_msgs::msg::Route::ConstSharedPtr last_route_;
  lanelet::ConstLanelets route_lanelets_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory_;

  // Callback
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void onLaneletMapBin(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  void onRoute(const autoware_planning_msgs::msg::Route::ConstSharedPtr msg);
  void onReferenceTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onPredictedTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  // Publisher
  autoware_utils::DebugPublisher debug_publisher_{this, "~/debug"};
  autoware_utils::ProcessingTimePublisher processing_time_publisher_{this};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  bool isDataTimeout();
  void onTimer();

  // Parameter
  NodeParam node_param_;
  Param param_;
  double vehicle_length_m_;

  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Core
  Input input_{};
  Output output_{};
  std::unique_ptr<LaneDepartureChecker> lane_departure_checker_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_{this};

  void checkLaneDeparture(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkTrajectoryDeviation(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Visualization
  visualization_msgs::msg::MarkerArray createMarkerArray() const;
};
}  // namespace lane_departure_checker

#endif  // LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_
