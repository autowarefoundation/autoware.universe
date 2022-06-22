// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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
#ifndef RECORDREPLAY_PLANNER_NODES__RECORDREPLAY_PLANNER_NODE_HPP_
#define RECORDREPLAY_PLANNER_NODES__RECORDREPLAY_PLANNER_NODE_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <recordreplay_planner_nodes/visibility_control.hpp>
#include <recordreplay_planner/recordreplay_planner.hpp>
#include <autoware_auto_planning_msgs/action/record_trajectory.hpp>
#include <autoware_auto_planning_msgs/action/replay_trajectory.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_planning_msgs/srv/modify_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <motion_common/motion_common.hpp>
#include <motion_common/config.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <common/types.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>

using autoware::common::types::float64_t;

namespace motion
{
namespace planning
{
namespace recordreplay_planner_nodes
{
using PlannerPtr = std::unique_ptr<motion::planning::recordreplay_planner::RecordReplayPlanner>;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_planning_msgs::srv::ModifyTrajectory;
using autoware_auto_planning_msgs::action::RecordTrajectory;
using autoware_auto_planning_msgs::action::ReplayTrajectory;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using Transform = geometry_msgs::msg::TransformStamped;
using motion::motion_common::Real;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/// \brief ROS Node for recording and replaying trajectories
class RECORDREPLAY_PLANNER_NODES_PUBLIC RecordReplayPlannerNode : public rclcpp::Node
{
public:
  using GoalHandleRecordTrajectory = rclcpp_action::ServerGoalHandle<RecordTrajectory>;
  using GoalHandleReplayTrajectory = rclcpp_action::ServerGoalHandle<ReplayTrajectory>;

  /// Parameter file constructor
  explicit RecordReplayPlannerNode(const rclcpp::NodeOptions & node_options);

protected:
  rclcpp_action::Server<RecordTrajectory>::SharedPtr m_recordserver;
  rclcpp_action::Server<ReplayTrajectory>::SharedPtr m_replayserver;
  // May be nullptr if disabled
  rclcpp::Client<ModifyTrajectory>::SharedPtr m_modify_trajectory_client;
  std::shared_ptr<GoalHandleRecordTrajectory> m_recordgoalhandle;
  std::shared_ptr<GoalHandleReplayTrajectory> m_replaygoalhandle;

  rclcpp::Subscription<State>::SharedPtr m_ego_sub{};
  rclcpp::Publisher<Trajectory>::SharedPtr m_trajectory_pub{};
  rclcpp::Publisher<MarkerArray>::SharedPtr m_trajectory_viz_pub{};
  PlannerPtr m_planner{nullptr};

private:
  /// \brief Converts a TrajectoryPoint to a Marker for visualization
  /// \param[in] traj_point The TrajectoryPoint
  /// \param[in] frame_id The name of the frame of reference in which the marker should be placed
  /// \param[in] index A sequential index. When combined with namespace, makes a unique id
  /// \param[in] ns The namespace for the marker
  /// \returns A visualization_msgs::msg::Marker
  RECORDREPLAY_PLANNER_NODES_LOCAL Marker to_marker(
    const TrajectoryPoint & traj_point,
    const std::string & frame_id,
    int32_t index,
    const std::string & ns);

  /// \brief Converts a Trajectory to a MarkerArray for visualization
  /// \param[in] traj The Trajectory
  /// \param[in] ns The namespace for the markers
  /// \returns A visaulization_msgs::msg::MarkerArray
  RECORDREPLAY_PLANNER_NODES_LOCAL MarkerArray to_markers(
    const Trajectory & traj, const std::string & ns);

  /// \brief Clears the list of recorded markers
  RECORDREPLAY_PLANNER_NODES_LOCAL void clear_recorded_markers();

  RECORDREPLAY_PLANNER_NODES_LOCAL void on_ego(const State::SharedPtr & msg);
  RECORDREPLAY_PLANNER_NODES_LOCAL void modify_trajectory_response(
    rclcpp::Client<ModifyTrajectory>::SharedFuture future);

  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::GoalResponse record_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const RecordTrajectory::Goal> goal);
  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::CancelResponse record_handle_cancel(
    const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle);
  RECORDREPLAY_PLANNER_NODES_LOCAL void record_handle_accepted(
    const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle);

  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::GoalResponse replay_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const ReplayTrajectory::Goal> goal);
  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::CancelResponse replay_handle_cancel(
    const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle);
  RECORDREPLAY_PLANNER_NODES_LOCAL void replay_handle_accepted(
    const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle);

  std::string m_odom_frame_id{};
  MarkerArray m_recorded_markers{};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  float64_t m_goal_distance_threshold_m = {};
  float64_t m_goal_angle_threshold_rad;
  std::string m_recording_frame = "map";

  bool8_t m_enable_loop = false;
  float64_t m_max_loop_gap_m = 0.0;
};  // class RecordReplayPlannerNode
}  // namespace recordreplay_planner_nodes
}  // namespace planning
}  // namespace motion

#endif  // RECORDREPLAY_PLANNER_NODES__RECORDREPLAY_PLANNER_NODE_HPP_
