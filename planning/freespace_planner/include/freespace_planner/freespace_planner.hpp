// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_
#define FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <motion_common/motion_common.hpp>
#include <astar_search/astar_search.hpp>
#include <vehicle_constants_manager/vehicle_constants_manager.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/action/planner_costmap.hpp>
#include <autoware_auto_planning_msgs/action/plan_trajectory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <freespace_planner/visibility_control.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>


namespace autoware
{
namespace planning
{
namespace freespace_planner
{

enum class FreespacePlannerState
{
  IDLE,
  PLANNING
};


struct NodeParam
{
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
};

/// \class FreespacePlannerNode
/// \brief Creates Hybrid A* planning algorithm node
class FreespacePlannerNode : public rclcpp::Node
{
public:
  /// \brief Default constructor for FreespacePlannerNode class
  /// \param [in] node_options A rclcpp::NodeOptions object passed on to rclcpp::Node
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  using PlanTrajectoryAction = autoware_auto_planning_msgs::action::PlanTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanTrajectoryAction>;
  using PlannerCostmapAction = autoware_auto_planning_msgs::action::PlannerCostmap;
  using PlannerCostmapGoalHandle = rclcpp_action::ClientGoalHandle<PlannerCostmapAction>;

  // ros communication
  rclcpp_action::Client<PlannerCostmapAction>::SharedPtr map_client_;
  rclcpp_action::Server<PlanTrajectoryAction>::SharedPtr plan_trajectory_srv_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_trajectory_debug_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  astar_search::AstarParam astar_param_;
  FreespacePlannerState state_;

  // variables
  std::unique_ptr<astar_search::AstarSearch> astar_;
  std::shared_ptr<GoalHandle> planning_goal_handle_{nullptr};
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  autoware_auto_planning_msgs::msg::Trajectory trajectory_;
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;

  // callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const PlanTrajectoryAction::Goal>);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void goalResponseCallback(std::shared_future<PlannerCostmapGoalHandle::SharedPtr> future);
  void feedbackCallback(
    PlannerCostmapGoalHandle::SharedPtr,
    const std::shared_ptr<const PlannerCostmapAction::Feedback>) {}
  void resultCallback(const PlannerCostmapGoalHandle::WrappedResult & result);

  // functions
  void reset();
  bool isPlanning() const;
  void startPlanning();
  void stopPlanning();
  bool planTrajectory();
  geometry_msgs::msg::TransformStamped getTransform(
    const std::string & from, const std::string & to);

  void visualizeTrajectory();
};

}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware

#endif  // FREESPACE_PLANNER__FREESPACE_PLANNER_HPP_
