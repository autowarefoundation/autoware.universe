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

#ifndef SCENE_INTERSECTION_HPP_
#define SCENE_INTERSECTION_HPP_

#include "util_type.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <motion_utils/motion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace behavior_velocity_planner
{
// first: time, second: distance
using TimeDistanceArray = std::vector<std::pair<double, double>>;

class IntersectionModule : public SceneModuleInterface
{
public:
  struct DebugData
  {
    geometry_msgs::msg::Pose collision_stop_wall_pose;
    geometry_msgs::msg::Pose occlusion_stop_wall_pose;
    geometry_msgs::msg::Pose occlusion_first_stop_wall_pose;
    geometry_msgs::msg::Pose pass_judge_wall_pose;
    geometry_msgs::msg::Polygon stuck_vehicle_detect_area;
    geometry_msgs::msg::Polygon candidate_collision_ego_lane_polygon;
    std::vector<geometry_msgs::msg::Polygon> candidate_collision_object_polygons;
    std::vector<lanelet::ConstLanelet> intersection_detection_lanelets;
    std::vector<lanelet::CompoundPolygon3d> detection_area;
    geometry_msgs::msg::Polygon intersection_area;
    lanelet::CompoundPolygon3d ego_lane;
    std::vector<lanelet::CompoundPolygon3d> adjacent_area;
    autoware_auto_perception_msgs::msg::PredictedObjects conflicting_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects stuck_targets;
    geometry_msgs::msg::Pose predicted_obj_pose;
    geometry_msgs::msg::Point nearest_occlusion_point;
    geometry_msgs::msg::Point nearest_occlusion_projection_point;
  };

  struct PlannerParam
  {
    struct Common
    {
      double detection_area_margin;        //! used for detecting objects in detection area
      double detection_area_right_margin;  //! used for detecting objects in detection area only
                                           //! right direction
      double detection_area_left_margin;  //! used for detecting objects in detection area only left
                                          //! direction
      double detection_area_length;       //! used to create detection area polygon
      double detection_area_angle_thr;    //! threshold in checking the angle of detecting objects
      double stop_line_margin;  //! distance from auto-generated stopline to detection_area boundary
      double intersection_velocity;  //! used for intersection passing time
      double intersection_max_acc;   //! used for calculating intersection velocity
      double stop_overshoot_margin;  //! overshoot margin for stuck, collision detection
      bool use_intersection_area;
      double path_interpolation_ds;
    } common;
    struct StuckVehicle
    {
      bool use_stuck_stopline;  //! stopline generate before the intersection lanelet when is_stuck.
      double stuck_vehicle_detect_dist;  //! distance from end point to finish stuck vehicle check
      double stuck_vehicle_ignore_dist;  //! distance from intersection start to start stuck vehicle
                                         //! check
      double stuck_vehicle_vel_thr;      //! Threshold of the speed to be recognized as stopped
      double
        assumed_front_car_decel;  //! the expected deceleration of front car when front car as well
      bool enable_front_car_decel_prediction;  //! flag for using above feature
    } stuck_vehicle;
    struct CollisionDetection
    {
      double state_transit_margin_time;
      double min_predicted_path_confidence;
      //! minimum confidence value of predicted path to use for collision detection
      double minimum_ego_predicted_velocity;  //! used to calculate ego's future velocity profile
      double collision_start_margin_time;     //! start margin time to check collision
      double collision_end_margin_time;       //! end margin time to check collision
      double keep_detection_vel_thr;  //! keep detection if ego is ego.vel < keep_detection_vel_thr
    } collision_detection;
    struct Occlusion
    {
      bool enable;
      double occlusion_detection_area_length;  //! used for occlusion detection
      bool enable_creeping;
      double occlusion_creep_velocity;  //! the creep velocity to occlusion limit stop line
      double peeking_offset;
      int free_space_max;
      int occupied_min;
      bool do_dp;
      double before_creep_stop_time;
      double min_vehicle_brake_for_rss;
      double max_vehicle_velocity_for_rss;
      double denoise_kernel;
      bool pub_debug_grid;
    } occlusion;
  };

  enum OcclusionState {
    NONE,
    BEFORE_FIRST_STOP_LINE,
    WAIT_FIRST_STOP_LINE,
    CREEP_SECOND_STOP_LINE,
    COLLISION_DETECTED,
  };

  using Indecisive = std::monostate;
  struct StuckStop
  {
    size_t stop_line_idx;
    util::IntersectionStopLines stop_lines;
  };
  struct NonOccludedCollisionStop
  {
    size_t stop_line_idx;
    util::IntersectionStopLines stop_lines;
  };
  struct FirstWaitBeforeOcclusion
  {
    size_t first_stop_line_idx;
    size_t occlusion_stop_line_idx;
    OcclusionState occlusion_state;
    util::IntersectionStopLines stop_lines;
  };
  struct PeekingTowardOcclusion
  {
    size_t stop_line_idx;
    std::optional<std::pair<size_t, size_t>> creep_interval;
    OcclusionState occlusion_state;
    util::IntersectionStopLines stop_lines;
  };
  struct OccludedCollisionStop
  {
    size_t stop_line_idx;
    size_t occlusion_stop_line_idx;
    OcclusionState occlusion_state;
    util::IntersectionStopLines stop_lines;
  };
  struct Safe
  {
    // if RTC is disapproved status, default stop lines are needed.
    util::IntersectionStopLines stop_lines;
  };
  using DecisionResult = std::variant<
    Indecisive, NonOccludedCollisionStop, FirstWaitBeforeOcclusion, PeekingTowardOcclusion,
    OccludedCollisionStop, Safe>;

  IntersectionModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<int> & associative_ids,
    const bool enable_occlusion_detection, rclcpp::Node & node, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

  const std::set<int> & getAssocIds() const { return associative_ids_; }

  UUID getOcclusionUUID() const { return occlusion_uuid_; }
  bool getOcclusionSafety() const { return occlusion_safety_; }
  double getOcclusionDistance() const { return occlusion_stop_distance_; }
  void setOcclusionActivation(const bool activation) { occlusion_activated_ = activation; }
  bool isOccluded() const { return is_actually_occluded_ || is_forcefully_occluded_; }
  bool isOcclusionFirstStopRequired() { return occlusion_first_stop_required_; }

private:
  rclcpp::Node & node_;
  const int64_t lane_id_;
  std::string turn_direction_;
  bool is_go_out_ = false;
  // Parameter
  PlannerParam planner_param_;
  std::optional<util::IntersectionLanelets> intersection_lanelets_;
  // for an intersection lane, its associative lanes are those that share same parent lanelet and
  // have same turn_direction
  const std::set<int> associative_ids_;

  // for occlusion detection
  const bool enable_occlusion_detection_;
  std::optional<std::vector<util::DescritizedLane>> detection_divisions_;
  bool is_actually_occluded_ = false;    //! occlusion based on occupancy_grid
  bool is_forcefully_occluded_ = false;  //! fake occlusion forced by external operator
  OcclusionState prev_occlusion_state_ = OcclusionState::NONE;
  // NOTE: uuid_ is base member
  // for occlusion clearance decision
  const UUID occlusion_uuid_;
  bool occlusion_safety_ = true;
  double occlusion_stop_distance_;
  bool occlusion_activated_ = true;
  // for first stop in two-phase stop
  const UUID occlusion_first_stop_uuid_;
  bool occlusion_first_stop_required_ = false;

  StateMachine collision_state_machine_;     //! for stable collision checking
  StateMachine before_creep_state_machine_;  //! for two phase stop

  void initializeRTCStatus();

  DecisionResult modifyPathVelocityDetail(PathWithLaneId * path, StopReason * stop_reason);

  std::optional<std::pair<size_t, bool>> checkStuckVehicle(
    const std::shared_ptr<const PlannerData> & planner_data,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::CompoundPolygon3d & first_conflicting_area,
    autoware_auto_planning_msgs::msg::PathWithLaneId * input_path);

  bool isOcclusionCleared(
    const nav_msgs::msg::OccupancyGrid & occ_grid,
    const std::vector<lanelet::CompoundPolygon3d> & detection_areas,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const lanelet::CompoundPolygon3d & first_detection_area,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const std::vector<util::DescritizedLane> & lane_divisions,
    const double occlusion_dist_thr) const;

  // Debug
  DebugData debug_data_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr occlusion_grid_pub_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_INTERSECTION_HPP_
