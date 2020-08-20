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

#include <boost/optional.hpp>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>
#include "utilization/boost_geometry_helper.h"

struct BlindSpotPolygons
{
  lanelet::CompoundPolygon3d conflict_area;
  lanelet::CompoundPolygon3d detection_area;
};

class BlindSpotModule : public SceneModuleInterface
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

  enum class TurnDirection { LEFT = 0, RIGHT, INVALID };

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
    lanelet::CompoundPolygon3d confict_area_for_blind_spot;
    lanelet::CompoundPolygon3d detection_area_for_blind_spot;
    autoware_planning_msgs::PathWithLaneId spline_path;
    autoware_perception_msgs::DynamicObjectArray conflicting_targets;
  };

public:
  struct PlannerParam
  {
    double stop_line_margin;  //! distance from auto-generated stopline to detection_area boundary
    double
      backward_length;  //! distance[m] from closest path point to the edge of beginning point in area
    double
      max_future_movement_time;  //! maximum time[second] for considering future movement of object
  };

  BlindSpotModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param);

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
  TurnDirection turn_direction_;
  bool has_traffic_light_;

  // Parameter
  PlannerParam planner_param_;

  /**
   * @brief Check obstacle is in blind spot areas.
   * Condition1: Object's position is in broad blind spot area.
   * Condition2: Object's predicted postition is in narrow blind spot area.
   * If both coditions are met, return true
   * @param path path information associated with lane id
   * @param objects_ptr dynamic objects
   * @param closest_idx closest path point index from ego car in path points
   * @return true when an object is detected in blind spot
   */
  bool checkObstacleInBlindSpot(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const autoware_planning_msgs::PathWithLaneId & path,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr,
    const int closest_idx) const;

  /**
   * @brief Create half lanelet
   * @param lanelet input lanelet
   * @return Half lanelet
   */
  lanelet::ConstLanelet generateHalfLanelet(const lanelet::ConstLanelet lanelet) const;

  /**
   * @brief Make blind spot areas. Narrow area is made from closest path point to stop line index.
   * Broad area is made from backward expanded point to stop line point
   * @param path path information associated with lane id
   * @param closest_idx closest path point index from ego car in path points
   * @return Blind spot polygons
   */
  BlindSpotPolygons generateBlindSpotPolygons(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx) const;

  /**
   * @brief Get vehicle edge
   * @param vehicle_pose pose of ego vehicle
   * @param vehicle_width width of ego vehicle
   * @param base_link2front length between base link and front of ego vehicle
   * @return edge of ego vehicle
   */
  lanelet::LineString2d getVehicleEdge(
    const geometry_msgs::Pose & vehicle_pose, const double vehicle_width,
    const double base_link2front) const;

  /**
   * @brief Check if object is belong to targeted classes
   * @param object Dynamic object
   * @return True when object belong to targeted classes
   */
  bool isTargetObjectType(const autoware_perception_msgs::DynamicObject & object) const;

  /**
   * @brief Check if at least one of object's predicted poistion is in area
   * @param object Dynamic object
   * @param area Area defined by polygon
   * @return True when at least one of object's predicted position is in area
   */
  bool isPredictedPathInArea(
    const autoware_perception_msgs::DynamicObject & object,
    const lanelet::CompoundPolygon3d & area) const;

  /**
   * @brief Generate a stop line and insert it into the path.
   * A stop line is at an intersection point of straight path with vehicle path
   * @param detection_areas used to generate stop line
   * @param path            ego-car lane
   * @param stop_line_idx   generated stop line index
   * @param pass_judge_line_idx  generated pass judge line index
   * @return false when generation failed
   */
  bool generateStopLine(
    const lanelet::ConstLanelets straight_lanelets, autoware_planning_msgs::PathWithLaneId * path,
    int * stop_line_idx, int * pass_judge_line_idx) const;

  /**
   * @brief Calculate first path index that is conflicting lanelets.
   * @param path     target path
   * @param laneletss target lanelets
   * @return path point index
   */
  boost::optional<int> getFirstPointConflictingLanelets(
    const autoware_planning_msgs::PathWithLaneId & path,
    const lanelet::ConstLanelets & lanelets) const;

  /**
   * @brief Get start point from lanelet
   * @param lane_id lane id of objective lanelet
   * @return end point of lanelet
   */
  boost::optional<geometry_msgs::Point> getStartPointFromLaneLet(const int lane_id) const;

  /**
   * @brief get straight lanelets in intersection
   */
  lanelet::ConstLanelets getStraightLanelets(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr, const int lane_id);

  /**
   * @brief Modify objects predicted path. remove path point if the time exceeds timer_thr.
   * @param objects_ptr target objects
   * @param time_thr    time threshold to cut path
   */
  void cutPredictPathWithDuration(
    autoware_perception_msgs::DynamicObjectArray * objects_ptr, const double time_thr) const;

  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;
};
