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

#ifndef MRM_PULL_OVER_MANAGER__MRM_PULL_OVER_MANAGER_CORE_HPP_
#define MRM_PULL_OVER_MANAGER__MRM_PULL_OVER_MANAGER_CORE_HPP_

// C++
#include <map>
#include <memory>
#include <string>
#include <vector>

// ROS 2
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

// Autoware
#include <motion_utils/trajectory/trajectory.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <tier4_system_msgs/msg/emergency_goals_clear_command.hpp>
#include <tier4_system_msgs/msg/emergency_goals_stamped.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// lanelet
#include <lanelet2_extension/regulatory_elements/Forward.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace mrm_pull_over_manager
{
class MrmPullOverManager : public rclcpp::Node
{
public:
  MrmPullOverManager();

private:
  using Pose = geometry_msgs::msg::Pose;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using Odometry = nav_msgs::msg::Odometry;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using PoseLaneIdMap = std::map<lanelet::Id, Pose>;
  using MrmBehaviorStatus = tier4_system_msgs::msg::MrmBehaviorStatus;
  using EmergencyGoalsClearCommand = tier4_system_msgs::msg::EmergencyGoalsClearCommand;
  using EmergencyGoalsStamped = tier4_system_msgs::msg::EmergencyGoalsStamped;

  struct Parameter
  {
    double update_rate;
    std::string pull_over_point_file_path;
    size_t max_goal_pose_num;
    double yaw_deviation_threshold;
    double margin_time_to_goal;
  };

  // Subscribtoers
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;

  Odometry::ConstSharedPtr odom_;
  LaneletRoute::ConstSharedPtr route_;
  route_handler::RouteHandler route_handler_;
  Trajectory::ConstSharedPtr trajectory_;

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_route(const LaneletRoute::ConstSharedPtr msg);
  void on_map(const HADMapBin::ConstSharedPtr msg);
  void on_trajectory(const Trajectory::ConstSharedPtr msg);

  // Server
  rclcpp::Service<tier4_system_msgs::srv::OperateMrm>::SharedPtr operate_mrm_;

  void operateMrm(
    const tier4_system_msgs::srv::OperateMrm::Request::SharedPtr request,
    const tier4_system_msgs::srv::OperateMrm::Response::SharedPtr response);

  // Publisher
  rclcpp::Publisher<EmergencyGoalsStamped>::SharedPtr pub_emergency_goals_;
  rclcpp::Publisher<MrmBehaviorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<EmergencyGoalsClearCommand>::SharedPtr pub_emergency_goals_clear_command_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void on_timer();

  // Parameters
  Parameter param_;

  // Variables
  PoseLaneIdMap candidate_goals_;
  MrmBehaviorStatus status_;

  // Algorithm
  bool is_data_ready();
  void publish_status() const;
  void publish_emergency_goals_clear_command() const;
  void publish_emergency_goals(const std::vector<Pose> & emergency_goals) const;
  std::string get_module_name() const;

  /**
   * @brief Find the goals within the lanelet and publish them
   */
  bool find_goals_within_route();

  /**
   * @brief Find the goals that have the same lanelet id with the candidate_lanelets
   * @param candidate_lanelets
   * @return
   */
  std::vector<Pose> find_goals_in_lanelets(const lanelet::ConstLanelets & candidate_lanelets) const;

  /**
   * @brief Find the goals that have the same lanelet id with the candidate_lanelets
   * @param poses Poses to be filtered
   * @return Filtered poses
   */
  std::vector<Pose> filter_nearby_goals(const std::vector<Pose> & poses);
};
}  // namespace mrm_pull_over_manager

#endif  // MRM_PULL_OVER_MANAGER__MRM_PULL_OVER_MANAGER_CORE_HPP_
