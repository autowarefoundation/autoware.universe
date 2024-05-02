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

#ifndef OUT_OF_LANE_MODULE_HPP_
#define OUT_OF_LANE_MODULE_HPP_

#include "types.hpp"

#include <behavior_velocity_planner_common/velocity_factor_interface.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <motion_velocity_planner_common/plugin_module_interface.hpp>
#include <motion_velocity_planner_common/velocity_planning_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace motion_velocity_planner
{
class OutOfLaneModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  VelocityPlanningResult plan(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
    const PlannerData & planner_data) override;
  std::string get_module_name() const override { return module_name_; }

private:
  void init_parameters(rclcpp::Node & node);
  inline static const std::string module_name_ = "out_of_lane";
  out_of_lane::PlannerParam params_;

  std::optional<out_of_lane::SlowdownToInsert> prev_inserted_point_{};
  rclcpp::Clock::SharedPtr clock_{};
  rclcpp::Logger logger_ = rclcpp::get_logger("");
  rclcpp::Time prev_inserted_point_time_{};
  behavior_velocity_planner::VelocityFactor velocity_factor_;

protected:
  // Debug
  mutable out_of_lane::DebugData debug_data_;
};
}  // namespace motion_velocity_planner

#endif  // OUT_OF_LANE_MODULE_HPP_
