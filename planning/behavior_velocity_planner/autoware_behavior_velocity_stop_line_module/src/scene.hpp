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

#ifndef SCENE_HPP_
#define SCENE_HPP_
#define EIGEN_MPL2_ONLY

#include "autoware/behavior_velocity_planner_common/scene_module_interface.hpp"
#include "autoware/behavior_velocity_planner_common/utilization/util.hpp"
#include "autoware/motion_utils/factor/velocity_factor_interface.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <optional>
#include <utility>

namespace autoware::behavior_velocity_planner
{
class StopLineModule : public SceneModuleInterface
{
public:
  using StopLineWithLaneId = std::pair<lanelet::ConstLineString3d, int64_t>;
  using Trajectory =
    autoware::trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>;
  enum class State { APPROACH, STOPPED, START };

  struct DebugData
  {
    double base_link2front;
    std::optional<geometry_msgs::msg::Pose> stop_pose;
  };

  struct PlannerParam
  {
    double stop_margin;
    double stop_duration_sec;
    double hold_stop_margin_distance;
    bool use_initialization_stop_line_state;
  };

  StopLineModule(
    const int64_t module_id, lanelet::ConstLineString3d stop_line,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  std::pair<double, std::optional<double>> getEgoAndStopPoint(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose,
    const State & state) const;

  void updateStateAndStoppedTime(
    State * state, std::optional<rclcpp::Time> * stopped_time, const rclcpp::Time & now,
    const double & distance_to_stop_point, const bool & is_vehicle_stopped) const;

  static void updateVelocityFactor(
    autoware::motion_utils::VelocityFactorInterface * velocity_factor, const State & state,
    const double & distance_to_stop_point);

  void updateStopReason(StopReason * stop_reason, const geometry_msgs::msg::Pose & stop_pose) const;

  void updateDebugData(
    DebugData * debug_data, const geometry_msgs::msg::Pose & stop_pose, const State & state) const;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override
  {
    return visualization_msgs::msg::MarkerArray{};
  }
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  const lanelet::ConstLineString3d stop_line_;
  // Parameter
  const PlannerParam planner_param_;

  // State machine
  State state_;

  std::optional<rclcpp::Time> stopped_time_;

  // Debug
  DebugData debug_data_;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
