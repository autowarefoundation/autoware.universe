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

#include <scene_module/scene_module_interface.h>
#include "utilization/boost_geometry_helper.h"

class IntersectionModule : public SceneModuleInterface
{
public:
  enum class State {
    STOP = 0,
    GO,
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
    autoware_perception_msgs::DynamicObjectArray conflicting_targets;
    autoware_perception_msgs::DynamicObjectArray stuck_targets;
  };

public:
  struct PlannerParam
  {
    double state_transit_mergin_time;
    double decel_velocoity;    //! used when in straight and traffic_light lane
    double path_expand_width;  //! path width to calculate the edge line for both side
    double stop_line_margin;   //! distance from auto-generated stopline to detection_area boundary
    double
      stuck_vehicle_detect_dist;  //! distance from intersection end point to finish stuck vehicle check
    double
      stuck_vehicle_ignore_dist;  //! distance from intersection start point to start stuck vehicle check
    double stuck_vehicle_vel_thr;  //! Threshold of the speed to be recognized as stopped
    double intersection_velocity;  //! used for intersection passing time
    double detection_area_length;  //! used to create detection area polygon
  };

  IntersectionModule(
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
  std::string turn_direction_;
  bool has_traffic_light_;

  // Parameter
  PlannerParam planner_param_;

  /**
   * @brief check collision for all lanelet area & dynamic objects (call checkPathCollision() as
   * actual collision check algorithm inside this function)
   * @param path             ego-car lane
   * @param detection_areas  collidion check is performed for vehicles that exist in this area
   * @param objects_ptr      target objects
   * @param closest_idx      ego-car position index on the lane
   * @return true if collision is detected
   */
  bool checkCollision(
    const autoware_planning_msgs::PathWithLaneId & path,
    const std::vector<lanelet::CompoundPolygon3d> & detection_areas,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr,
    const int closest_idx);

  /**
   * @brief Check if there is a stopped vehicle on the ego-lane.
   * @param path            ego-car lane
   * @param closest_idx     ego-car position on the lane
   * @param objects_ptr     target objects
   * @return true if exists
   */
  bool checkStuckVehicleInIntersection(
    const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx, const int stop_idx,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr) const;

  /**
   * @brief Calculate the polygon of the path from the ego-car position to the end of the
   * intersection lanelet (+ extra distance).
   * @param path           ego-car lane
   * @param closest_idx    ego-car position index on the lane
   * @param extra_dist     extra distance from the end point of the intersection lanelet
   * @param ignore_dist    ignore distance from the start point of the ego-intersection lane
   * @return generated polygon
   */
  Polygon2d generateEgoIntersectionLanePolygon(
    const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx, const int start_idx,
    const double extra_dist, const double ignore_dist) const;

  /**
   * @brief Modify objects predicted path. remove path point if the time exceeds timer_thr.
   * @param objects_ptr target objects
   * @param time_thr    time threshold to cut path
   */
  void cutPredictPathWithDuration(
    autoware_perception_msgs::DynamicObjectArray * objects_ptr, const double time_thr) const;

  /**
   * @brief Calculate time that is needed for ego-vehicle to cross the intersection. (to be updated)
   * @param path              ego-car lane
   * @param closest_idx       ego-car position index on the lane
   * @param objective_lane_id lanelet id on ego-car
   * @return calculated time [s]
   */
  double calcIntersectionPassingTime(
    const autoware_planning_msgs::PathWithLaneId & path, const int closest_idx,
    const int objective_lane_id) const;

  /**
   * @brief check if the object has a terget type
   * @param object target object
   * @return true if the object has a target type
   */
  bool isTargetVehicleType(const autoware_perception_msgs::DynamicObject & object) const;

  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;
};
