// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef SCENE_MODULE__SPEED_BUMP__SCENE_HPP_
#define SCENE_MODULE__SPEED_BUMP__SCENE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>

namespace behavior_velocity_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point32;

class SpeedBumpModule : public SceneModuleInterface
{
public:
  enum class State { SLOW_DOWN, INSIDE, OUT };

  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::msg::Pose> slow_start_poses;
    std::vector<geometry_msgs::msg::Point> slow_end_points;
    std::vector<geometry_msgs::msg::Point> path_polygon_intersection_points;
    std::vector<geometry_msgs::msg::Point> speed_bump_polygon;
  };

  struct PlannerParam
  {
    double slow_start_margin;
    double slow_end_margin;
    bool print_debug_info;
  };

  SpeedBumpModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::SpeedBump & speed_bump_reg_elem, const PlannerParam & planner_param,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  int64_t module_id_;
  int64_t lane_id_;

  // Speed Bump Regulatory Element
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem_;

  // Parameter
  PlannerParam planner_param_;

  // State
  State state_;

  // Debug
  DebugData debug_data_;

  std::vector<geometry_msgs::msg::Point> path_intersects_;

  bool applySlowDownSpeed(PathWithLaneId & output);

  void insertDecelPointWithDebugInfo(
    const geometry_msgs::msg::Point & slow_point, const float target_velocity,
    PathWithLaneId & output);

  // returns m and b consts for y=mx+b
  static std::pair<float, float> getLinearEquation(Point32 p1, Point32 p2);

  bool passed_slow_start_point_;
  float speed_bump_height_;
  float speed_bump_slow_down_speed_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__SPEED_BUMP__SCENE_HPP_
