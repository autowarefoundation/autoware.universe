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

    /**
     * @brief set request state command with margin time
     */
    void setStateWithMarginTime(State state);

    /**
     * @brief set request state command directly
     */
    void setState(State state);

    /**
     * @brief set margin time
     */
    void setMarginTime(const double t);

    /**
     * @brief get current state
     */
    State getState();

  private:
    State state_;         //! current state
    double margin_time_;  //! margin time when transit to Go from Stop
    std::shared_ptr<ros::Time>
      start_time_;  //! timer start time when received Go state when current state is Stop
  };

  struct DebugData
  {
    autoware_planning_msgs::PathWithLaneId path_raw;

    geometry_msgs::Pose virtual_wall_pose;
    geometry_msgs::Pose stop_point_pose;
    geometry_msgs::Pose judge_point_pose;
    autoware_planning_msgs::PathWithLaneId path_with_judgeline;
    std::vector<lanelet::ConstLanelet> intersection_detection_lanelets;
    autoware_planning_msgs::PathWithLaneId path_right_edge;
    autoware_planning_msgs::PathWithLaneId path_left_edge;
  };

public:
  IntersectionModule(const int64_t module_id, const int64_t lane_id);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;

private:
  int64_t lane_id_;

  int stop_line_idx_;   //! stop-line index
  int judge_line_idx_;  //! stop-judgement-line index

  // Parameter
  double judge_line_dist_ = 0.0;  //! distance from stop-line to stop-judgement line
  double approaching_speed_to_stopline_ =
    100.0 / 3.6;                    //! speed when approaching stop-line (should be slow)
  double path_expand_width_ = 2.0;  //! path width to calculate the edge line for both side
  bool show_debug_info_ = false;

  /**
   * @brief set velocity from idx to the end point
   */
  bool setVelocityFrom(
    const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId * input);

  /**
   * @brief get objective lanelets for detection area
   */
  bool getObjectiveLanelets(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr, const int lane_id,
    std::vector<lanelet::ConstLanelet> * objective_lanelets);

  /**
   * @brief check collision with path & dynamic object predicted path
   */
  bool checkPathCollision(
    const autoware_planning_msgs::PathWithLaneId & path,
    const autoware_perception_msgs::DynamicObject & object);

  /**
   * @brief check collision for all lanelet area & dynamic objects (call checkPathCollision() as
   * actual collision check algorithm inside this function)
   */
  bool checkCollision(
    const autoware_planning_msgs::PathWithLaneId & path,
    const std::vector<lanelet::ConstLanelet> & objective_lanelets,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr,
    const double path_width);

  /**
   * @brief calculate right and left path edge line
   */
  bool generateEdgeLine(
    const autoware_planning_msgs::PathWithLaneId & path, const double path_width,
    autoware_planning_msgs::PathWithLaneId * path_r,
    autoware_planning_msgs::PathWithLaneId * path_l);

  /**
   * @brief set stop-line and stop-judgement-line index. This may modificates path size due to
   * interpolate insertion.
   */
  bool setStopLineIdx(
    const int closest, const double judge_line_dist, autoware_planning_msgs::PathWithLaneId * path,
    int * stop_line_idx, int * judge_line_idx);

  geometry_msgs::Pose getAheadPose(
    const size_t start_idx, const double ahead_dist,
    const autoware_planning_msgs::PathWithLaneId & path) const;

  StateMachine state_machine_;  //! for state

  // Debug
  DebugData debug_data_;
};
