// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_SEARCH_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_SEARCH_HPP_

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/reeds_shepp.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
enum class NodeStatus : uint8_t { None, Open, Closed };

struct AstarParam
{
  // base configs
  bool only_behind_solutions;  // solutions should be behind the goal
  bool use_back;         // backward search
  double expansion_distance;

  // search configs
  double distance_heuristic_weight;  // obstacle threshold on grid [0,255]
  double steering_change_weight;
};

struct AstarNode
{
  NodeStatus status = NodeStatus::None;  // node status
  double x;                              // x
  double y;                              // y
  double theta;                          // theta
  double gc = 0;                         // actual motion cost
  double fc = 0;                         // total node cost
  double dir_distance = 0;               // distance travelled from last direction change
  int steering_index;                    // steering index
  bool is_back;                          // true if the current direction of the vehicle is back
  AstarNode * parent = nullptr;          // parent node
};

struct NodeComparison
{
  bool operator()(const AstarNode * lhs, const AstarNode * rhs)
  {
    return lhs->fc > rhs->fc;
  }
};

struct NodeUpdate
{
  double shift_x;
  double shift_y;
  double shift_theta;
  double distance;
  int steering_index;
  bool is_back;

  NodeUpdate rotated(const double theta) const
  {
    NodeUpdate result = *this;
    result.shift_x = std::cos(theta) * this->shift_x - std::sin(theta) * this->shift_y;
    result.shift_y = std::sin(theta) * this->shift_x + std::cos(theta) * this->shift_y;
    return result;
  }

  NodeUpdate flipped() const
  {
    NodeUpdate result = *this;
    result.shift_y = -result.shift_y;
    result.shift_theta = -result.shift_theta;
    return result;
  }

  NodeUpdate reversed() const
  {
    NodeUpdate result = *this;
    result.shift_x = -result.shift_x;
    result.shift_theta = -result.shift_theta;
    result.is_back = !result.is_back;
    return result;
  }
};

class AstarSearch : public AbstractPlanningAlgorithm
{
public:
  using TransitionTable = std::vector<std::vector<NodeUpdate>>;

  AstarSearch(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    const AstarParam & astar_param);

  AstarSearch(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
    rclcpp::Node & node)
  : AstarSearch(
      planner_common_param, collision_vehicle_shape,
      AstarParam{
        node.declare_parameter<bool>("astar.only_behind_solutions"),
        node.declare_parameter<bool>("astar.use_back"),
        node.declare_parameter<double>("astar.expansion_distance"),
        node.declare_parameter<double>("astar.distance_heuristic_weight"),
        node.declare_parameter<double>("astar.steering_change_weight")})
  {
  }

  void setMap(const nav_msgs::msg::OccupancyGrid & costmap) override;
  bool makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose) override;

  const PlannerWaypoints & getWaypoints() const { return waypoints_; }

  inline int getKey(const IndexXYT & index)
  {
    return indexToId(index) * planner_common_param_.theta_size + index.theta;
  }

private:
  void setTransitionTable();
  bool search();
  void expandNodes(AstarNode & current_node);
  void clearNodes();
  void setPath(const AstarNode & goal);
  bool setStartNode();
  bool setGoalNode();
  double estimateCost(const geometry_msgs::msg::Pose & pose) const;
  bool isGoal(const AstarNode & node) const;
  geometry_msgs::msg::Pose node2pose(const AstarNode & node) const;

  double getSteeringCost(const int steering_index) const;
  double getSteeringChangeCost(const int steering_index, const int prev_steering_index) const;

  // Algorithm specific param
  AstarParam astar_param_;

  // hybrid astar variables
  TransitionTable transition_table_;
  std::vector<AstarNode> graph_;

  std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison> openlist_;

  // goal node, which may helpful in testing and debugging
  AstarNode * goal_node_;

  // distance metric option (removed when the reeds_shepp gets stable)
  bool use_reeds_shepp_;

  double steering_resolution_;
  double heading_resolution_;
  double avg_turning_radius_;

  // threshold for minimum distance between direction switches
  static constexpr double min_dir_change_dist_ = 1.5;
};
}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ASTAR_SEARCH_HPP_
