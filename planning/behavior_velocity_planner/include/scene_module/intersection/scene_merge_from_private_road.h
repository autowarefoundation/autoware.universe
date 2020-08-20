/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/intersection/scene_intersection.h>
#include <scene_module/scene_module_interface.h>
#include "utilization/boost_geometry_helper.h"

/**
 * @brief This module makes sure that vehicle will stop before entering public road from private road.
 *        This module is meant to be regesistered with intersection module, which looks at intersecting lanes
 *        before entering intersection
 */

class MergeFromPrivateRoadModule : public SceneModuleInterface
{
public:
  enum class State {
    STOP = 0,
    GO,
  };
  std::string toString(const State & state)
  {
    if (state == State::STOP)
      return "STOP";
    else if (state == State::GO)
      return "GO";
    else
      return "UNKNOWN";
  };

  /**
   * @brief Manage stop-go states with safety margin time.
   */
  class StateMachine
  {
  public:
    StateMachine()
    {
      state_ = State::GO;
      margin_time_ = 0.0;
    }
    void setStateWithMarginTime(State state);
    void setState(State state);
    void setMarginTime(const double t);
    State getState();

  private:
    State state_;                            //! current state
    double margin_time_;                     //! margin time when transit to Go from Stop
    std::shared_ptr<ros::Time> start_time_;  //! first time received GO when STOP state
  };

  struct DebugData
  {
    autoware_planning_msgs::PathWithLaneId path_raw;

    geometry_msgs::Pose virtual_wall_pose;
    geometry_msgs::Pose stop_point_pose;
    geometry_msgs::Pose judge_point_pose;
    geometry_msgs::Polygon ego_lane_polygon;
    geometry_msgs::Polygon stuck_vehicle_detect_area;
    std::vector<lanelet::ConstLanelet> intersection_detection_lanelets;
    std::vector<lanelet::CompoundPolygon3d> detection_area;
    autoware_planning_msgs::PathWithLaneId spline_path;
    geometry_msgs::Point first_collision_point;
    autoware_perception_msgs::DynamicObjectArray stuck_targets;
  };

public:
  MergeFromPrivateRoadModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const IntersectionModule::PlannerParam & planner_param);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(
    autoware_planning_msgs::PathWithLaneId * path,
    autoware_planning_msgs::StopReason * stop_reason) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;

private:
  int64_t lane_id_;
  std::string turn_direction_;
  bool has_traffic_light_;

  // Parameter
  IntersectionModule::PlannerParam planner_param_;

  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;
};
