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
#pragma once

#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.hpp>

class StopLineModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, STOP, START };

  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::msg::Pose> stop_poses;
    geometry_msgs::msg::Pose first_stop_pose;
  };

  struct PlannerParam
  {
    double stop_margin;
    double stop_check_dist;
  };

public:
  StopLineModule(
    const int64_t module_id, const lanelet::ConstLineString3d & stop_line,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

private:
  int64_t module_id_;

  bool getBackwordPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);

  geometry_msgs::msg::Point getCenterOfStopLine(const lanelet::ConstLineString3d & stop_line);

  lanelet::ConstLineString3d stop_line_;
  State state_;

  // Paramter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
