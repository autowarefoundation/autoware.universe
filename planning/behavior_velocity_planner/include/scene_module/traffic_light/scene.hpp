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

#ifndef SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_
#define SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "boost/assert.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "rclcpp/rclcpp.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_routing/RoutingGraph.h"

#include "scene_module/scene_module_interface.hpp"

class TrafficLightModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, GO_OUT };
  enum class Input { PERCEPTION, EXTERNAL, NONE };  // EXTERNAL: FOA, V2X, etc.

  struct DebugData
  {
    double base_link2front;
    std::vector<std::tuple<
        std::shared_ptr<const lanelet::TrafficLight>,
        autoware_perception_msgs::msg::TrafficLightState>>
    tl_state;    // TODO(someone): replace tuple with struct
    std::vector<geometry_msgs::msg::Pose> stop_poses;
    geometry_msgs::msg::Pose first_stop_pose;
    std::vector<geometry_msgs::msg::Pose> dead_line_poses;
    std::vector<geometry_msgs::msg::Point> traffic_light_points;
  };

  struct PlannerParam
  {
    double stop_margin;
    double tl_state_timeout;
    double external_tl_state_timeout;
    bool enable_pass_judge;
  };

public:
  TrafficLightModule(
    const int64_t module_id, const lanelet::TrafficLight & traffic_light_reg_elem,
    lanelet::ConstLanelet lane, const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

  inline autoware_perception_msgs::msg::TrafficLightStateStamped getTrafficLightState() const
  {
    return tl_state_;
  }
  inline State getTrafficLightModuleState() const {return state_;}
  inline Input getTrafficLightModuleInput() const {return input_;}

private:
  int64_t lane_id_;

  bool getBackwardPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);

  bool insertTargetVelocityPoint(
    const autoware_planning_msgs::msg::PathWithLaneId & input,
    const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
    stop_line,
    const double & margin, const double & velocity,
    autoware_planning_msgs::msg::PathWithLaneId & output);

  bool getHighestConfidenceTrafficLightState(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_perception_msgs::msg::TrafficLightStateStamped & highest_confidence_tl_state);

  bool isOverDeadLine(
    const geometry_msgs::msg::Pose & self_pose,
    const autoware_planning_msgs::msg::PathWithLaneId & input_path,
    const size_t & dead_line_point_idx, const Eigen::Vector2d & dead_line_point,
    const double dead_line_range);

  bool isStopRequired(const autoware_perception_msgs::msg::TrafficLightState & tl_state);

  bool createTargetPoint(
    const autoware_planning_msgs::msg::PathWithLaneId & input,
    const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
    stop_line,
    const double & margin, size_t & target_point_idx, Eigen::Vector2d & target_point);

  bool hasLamp(
    const autoware_perception_msgs::msg::TrafficLightState & tl_state, const uint8_t & lamp_color);

  geometry_msgs::msg::Point getTrafficLightPosition(
    const lanelet::ConstLineStringOrPolygon3d traffic_light);

  bool getExternalTrafficLightState(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_perception_msgs::msg::TrafficLightStateStamped & external_tl_state);


  // Key Feature
  const lanelet::TrafficLight & traffic_light_reg_elem_;
  lanelet::ConstLanelet lane_;

  // State
  State state_;

  // Input
  Input input_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // prevent paththrough chattering
  bool is_prev_state_stop_;

  // Traffic Light State
  autoware_perception_msgs::msg::TrafficLightStateStamped tl_state_;
};
#endif  // SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_
