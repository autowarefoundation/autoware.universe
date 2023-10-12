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

#include <memory>
#include <string>
#include <utility>

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
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&MrmPullOverManager::onOdometry, this, _1));
  sub_map_ = create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MrmPullOverManager::onMap, this, _1));

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
}

void MrmPullOverManager::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void MrmPullOverManager::onMap(const HADMapBin::ConstSharedPtr msg)
{
  // lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  // lanelet::utils::conversion::fromBinMsg(
  //   *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  // const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
  //   lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  // const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
  //   lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  // lanelet::routing::RoutingGraphConstPtr vehicle_graph =
  //   lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  // lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
  //   lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  // lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  // overall_graphs_ptr_ =
  //   std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);

  route_handler_.setMap(*msg);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
}

bool MrmPullOverManager::isDataReady()
{
  // if (!hazard_status_stamped_) {
  //   RCLCPP_INFO_THROTTLE(
  //     this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
  //     "waiting for hazard_status_stamped msg...");
  //   return false;
  // }

  // if (
  //   param_.use_comfortable_stop && mrm_comfortable_stop_status_->state ==
  //                                    tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE) {
  //   RCLCPP_INFO_THROTTLE(
  //     this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
  //     "waiting for mrm comfortable stop to become available...");
  //   return false;
  // }

  // if (
  //   mrm_emergency_stop_status_->state ==
  //   tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE) { RCLCPP_INFO_THROTTLE(
  //     this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
  //     "waiting for mrm emergency stop to become available...");
  //   return false;
  // }

  return true;
}
