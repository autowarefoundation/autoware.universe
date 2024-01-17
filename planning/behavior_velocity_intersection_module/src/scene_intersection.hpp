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

#include "decision_result.hpp"
#include "interpolated_path_info.hpp"
#include "intersection_lanelets.hpp"
#include "intersection_stoplines.hpp"
#include "object_manager.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/string.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>

// TODO(Mamoru Sobue): forward as much as possible
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{

using TimeDistanceArray = std::vector<std::pair<double /* time*/, double /* distance*/>>;

/**
 * @struct
 * @brief see the document for more details of IntersectionStopLines
 */
struct TargetObject
{
  autoware_auto_perception_msgs::msg::PredictedObject object;
  std::optional<lanelet::ConstLanelet> attention_lanelet{std::nullopt};
  std::optional<lanelet::ConstLineString3d> stopline{std::nullopt};
  std::optional<double> dist_to_stopline{std::nullopt};
  void calc_dist_to_stopline();
};

/**
 * @struct
 * @brief categorize TargetObjects
 */
struct TargetObjects
{
  std_msgs::msg::Header header;
  std::vector<TargetObject> attention_objects;
  std::vector<TargetObject> parked_attention_objects;
  std::vector<TargetObject> intersection_area_objects;
  std::vector<TargetObject> all_attention_objects;  // TODO(Mamoru Sobue): avoid copy
};

class IntersectionModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    struct Common
    {
      double attention_area_length;
      double attention_area_margin;
      double attention_area_angle_threshold;
      bool use_intersection_area;
      double default_stopline_margin;
      double stopline_overshoot_margin;
      double path_interpolation_ds;
      double max_accel;
      double max_jerk;
      double delay_response_time;
      bool enable_pass_judge_before_default_stopline;
    } common;

    struct TurnDirection
    {
      bool left;
      bool right;
      bool straight;
    };

    struct StuckVehicle
    {
      TurnDirection turn_direction;
      bool use_stuck_stopline;
      double stuck_vehicle_detect_dist;
      double stuck_vehicle_velocity_threshold;
      bool disable_against_private_lane;
    } stuck_vehicle;

    struct YieldStuck
    {
      TurnDirection turn_direction;
      double distance_threshold;
    } yield_stuck;

    struct CollisionDetection
    {
      bool consider_wrong_direction_vehicle;
      double collision_detection_hold_time;
      double min_predicted_path_confidence;
      struct VelocityProfile
      {
        bool use_upstream;
        double minimum_upstream_velocity;
        double default_velocity;
        double minimum_default_velocity;
      } velocity_profile;
      struct FullyPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } fully_prioritized;
      struct PartiallyPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } partially_prioritized;
      struct NotPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } not_prioritized;
      struct YieldOnGreeTrafficLight
      {
        double distance_to_assigned_lanelet_start;
        double duration;
        double object_dist_to_stopline;
      } yield_on_green_traffic_light;
      struct IgnoreOnAmberTrafficLight
      {
        double object_expected_deceleration;
      } ignore_on_amber_traffic_light;
      struct IgnoreOnRedTrafficLight
      {
        double object_margin_to_path;
      } ignore_on_red_traffic_light;
    } collision_detection;

    struct Occlusion
    {
      bool enable;
      double occlusion_attention_area_length;
      int free_space_max;
      int occupied_min;
      double denoise_kernel;
      double attention_lane_crop_curvature_threshold;
      double attention_lane_curvature_calculation_ds;
      struct CreepDuringPeeking
      {
        bool enable;
        double creep_velocity;
      } creep_during_peeking;
      double peeking_offset;
      double occlusion_required_clearance_distance;
      std::vector<double> possible_object_bbox;
      double ignore_parked_vehicle_speed_threshold;
      double occlusion_detection_hold_time;
      double temporal_stop_time_before_peeking;
      bool temporal_stop_before_attention_area;
      double creep_velocity_without_traffic_light;
      double static_occlusion_with_traffic_light_timeout;
    } occlusion;

    struct Debug
    {
      std::vector<int64_t> ttc;
    } debug;
  };

  enum OcclusionType {
    //! occlusion is not detected
    NOT_OCCLUDED,
    //! occlusion is not caused by dynamic objects
    STATICALLY_OCCLUDED,
    //! occlusion is caused by dynamic objects
    DYNAMICALLY_OCCLUDED,
    //! actual occlusion does not exist, only disapproved by RTC
    RTC_OCCLUDED,
  };

  struct DebugData
  {
    std::optional<geometry_msgs::msg::Pose> collision_stop_wall_pose{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> occlusion_stop_wall_pose{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> occlusion_first_stop_wall_pose{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> first_pass_judge_wall_pose{std::nullopt};
    bool passed_first_pass_judge{false};
    bool passed_second_pass_judge{false};
    std::optional<geometry_msgs::msg::Pose> second_pass_judge_wall_pose{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> attention_area{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> occlusion_attention_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> ego_lane{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> adjacent_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> first_attention_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> second_attention_area{std::nullopt};
    std::optional<geometry_msgs::msg::Polygon> stuck_vehicle_detect_area{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> yield_stuck_detect_area{std::nullopt};
    std::optional<geometry_msgs::msg::Polygon> candidate_collision_ego_lane_polygon{std::nullopt};
    std::vector<geometry_msgs::msg::Polygon> candidate_collision_object_polygons;
    autoware_auto_perception_msgs::msg::PredictedObjects conflicting_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects amber_ignore_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects red_overshoot_ignore_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects stuck_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects yield_stuck_targets;
    std::vector<geometry_msgs::msg::Polygon> occlusion_polygons;
    std::optional<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>
      nearest_occlusion_projection{std::nullopt};
    autoware_auto_perception_msgs::msg::PredictedObjects blocking_attention_objects;
    std::optional<geometry_msgs::msg::Pose> absence_traffic_light_creep_wall{std::nullopt};
    std::optional<double> static_occlusion_with_traffic_light_timeout{std::nullopt};
  };

  /**
   * @struct
   * @brief categorize traffic light priority
   */
  enum class TrafficPrioritizedLevel {
    //! The target lane's traffic signal is red or the ego's traffic signal has an arrow.
    FULLY_PRIORITIZED = 0,
    //! The target lane's traffic signal is amber
    PARTIALLY_PRIORITIZED,
    //! The target lane's traffic signal is green
    NOT_PRIORITIZED
  };

  IntersectionModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
    const std::string & turn_direction, const bool has_traffic_light, rclcpp::Node & node,
    const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock);

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup primary-functions PRIMARY FUNCTIONS
   * @{
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;
  /** @}*/

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

  const std::set<lanelet::Id> & getAssociativeIds() const { return associative_ids_; }

  UUID getOcclusionUUID() const { return occlusion_uuid_; }
  bool getOcclusionSafety() const { return occlusion_safety_; }
  double getOcclusionDistance() const { return occlusion_stop_distance_; }
  void setOcclusionActivation(const bool activation) { occlusion_activated_ = activation; }
  bool isOcclusionFirstStopRequired() { return occlusion_first_stop_required_; }

private:
  rclcpp::Node & node_;

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup const-variables CONST VARIABLES
   * @{
   */
  //! lanelet of this intersection
  const lanelet::Id lane_id_;

  //! associative(sibling) lanelets ids
  const std::set<lanelet::Id> associative_ids_;

  //! turn_direction of this lane
  const std::string turn_direction_;

  //! flag if this intersection is traffic controlled
  const bool has_traffic_light_;

  //! RTC uuid for INTERSECTION_OCCLUSION
  const UUID occlusion_uuid_;
  /** @}*/

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup semi-const-variables SEMI CONST VARIABLES
   * @{
   */
  PlannerParam planner_param_;

  //! cache IntersectionLanelets struct
  std::optional<IntersectionLanelets> intersection_lanelets_{std::nullopt};

  //! cache discretized occlusion detection lanelets
  std::optional<std::vector<lanelet::ConstLineString3d>> occlusion_attention_divisions_{
    std::nullopt};
  /** @}*/

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup pass-judge-variable PASS JUDGE VARIABLES
   * @{
   */
  //! if true, this module never coomands to STOP anymore
  bool is_permanent_go_{false};

  //! for checking if ego is over the pass judge lines because previously the situation was SAFE
  DecisionResult prev_decision_result_{Indecisive{""}};

  //! flag if ego passed the 1st_pass_judge_line while peeking. If this is true, 1st_pass_judge_line
  //! is treated as the same position as occlusion_peeking_stopline
  bool passed_1st_judge_line_while_peeking_{false};

  //! save the time when ego passed the 1st/2nd_pass_judge_line with safe decision. If collision is
  //! expected after these variables are non-null, then it is the fault of past perception failure
  //! at these time.
  std::optional<rclcpp::Time> safely_passed_1st_judge_line_time_{std::nullopt};
  std::optional<rclcpp::Time> safely_passed_2nd_judge_line_time_{std::nullopt};
  /** @}*/

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup colision-variables COLLISION DETECTION VARIABLES
   * @{
   */
  //! debouncing for stable SAFE decision
  StateMachine collision_state_machine_;

  //! container for storing safety statuts of objects on the attention area
  ObjectInfoManager object_info_manager_;
  /** @} */

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup occlusion-variables OCCLUSION DETECTION VARIABLES
   * @{
   */
  OcclusionType prev_occlusion_status_;

  //! debouncing for the first brief stop at the default stopline
  StateMachine before_creep_state_machine_;

  //! debouncing for stable CLEARED decision
  StateMachine occlusion_stop_state_machine_;

  //! debouncing for the brief stop at the boundary of attention area(if required by the flag)
  StateMachine temporal_stop_before_attention_state_machine_;

  //! time counter for the stuck detection due to occlusion caused static objects
  StateMachine static_occlusion_timeout_state_machine_;
  /** @} */

  std::optional<rclcpp::Time> initial_green_light_observed_time_{std::nullopt};

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup RTC-variables RTC VARIABLES
   *
   * intersection module has additional rtc_interface_ for INTERSECTION_OCCLUSION in addition to the
   * default rtc_interface of SceneModuleManagerInterfaceWithRTC. activated_ is the derived member
   * of this module which is updated by the RTC config/service, so it should be read-only in this
   * module. occlusion_safety_ and occlusion_stop_distance_ are the corresponding RTC value for
   * INTERSECTION_OCCLUSION.
   * @{
   */
  bool occlusion_safety_{true};
  double occlusion_stop_distance_{0.0};
  bool occlusion_activated_{true};
  bool occlusion_first_stop_required_{false};
  /** @}*/

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @ingroup primary-functions
   * @{
   */
  /**
   * @brief set all RTC variable to safe and -INF
   */
  void initializeRTCStatus();

  /**
   * @ingroup primary-functions
   * @brief analyze traffic_light/occupancy/objects context and return DecisionResult
   */
  DecisionResult modifyPathVelocityDetail(PathWithLaneId * path, StopReason * stop_reason);

  /**
   * @ingroup primary-functions
   * @brief set RTC value according to calculated DecisionResult
   */
  void prepareRTCStatus(
    const DecisionResult &, const autoware_auto_planning_msgs::msg::PathWithLaneId & path);
  /** @}*/

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup prepare-data BASIC DATA CONSTRUCTION
   * @{
   */
  /**
   * @brief find the associated stopline road marking of assigned lanelet
   */
  std::optional<size_t> getStopLineIndexFromMap(
    const util::InterpolatedPathInfo & interpolated_path_info,
    lanelet::ConstLanelet assigned_lanelet);

  /**
   * @brief generate IntersectionStopLines
   */
  std::optional<IntersectionStopLines> generateIntersectionStopLines(
    lanelet::ConstLanelet assigned_lanelet,
    const lanelet::CompoundPolygon3d & first_conflicting_area,
    const lanelet::ConstLanelet & first_attention_lane,
    const std::optional<lanelet::CompoundPolygon3d> & second_attention_area_opt,
    const util::InterpolatedPathInfo & interpolated_path_info,
    autoware_auto_planning_msgs::msg::PathWithLaneId * original_path);

  /**
   * @brief generate IntersectionLanelets
   */
  IntersectionLanelets generateObjectiveLanelets(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const lanelet::ConstLanelet assigned_lanelet, const lanelet::ConstLanelets & lanelets_on_path);
  /** @} */

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup get-traffic-light TRAFFIC LIGHT RESPONSE
   * @{
   */
  /**
   * @brief check if associated traffic light is green
   */
  bool isGreenSolidOn(lanelet::ConstLanelet lane) const;

  /**
   * @brief find TrafficPrioritizedLevel
   */
  TrafficPrioritizedLevel getTrafficPrioritizedLevel(lanelet::ConstLanelet lane);
  /** @} */

  /**
   * @brief generate PathLanelets
   */
  std::optional<PathLanelets> generatePathLanelets(
    const lanelet::ConstLanelets & lanelets_on_path,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::CompoundPolygon3d & first_conflicting_area,
    const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
    const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx);

  /**
   * @brief check stuck
   */
  bool checkStuckVehicleInIntersection(const PathLanelets & path_lanelets, DebugData * debug_data);

  /**
   * @brief check yield stuck
   */
  bool checkYieldStuckVehicleInIntersection(
    const TargetObjects & target_objects, const util::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::ConstLanelets & attention_lanelets, DebugData * debug_data);

  /**
   * @brief categorize target objects
   */
  TargetObjects generateTargetObjects(
    const IntersectionLanelets & intersection_lanelets,
    const std::optional<Polygon2d> & intersection_area) const;

  /**
   * @brief update object info
   */
  void updateObjectInfoManager(
    const PathLanelets & path_lanelets, const TimeDistanceArray & time_distance_array,
    const TrafficPrioritizedLevel & traffic_prioritized_level,
    const IntersectionLanelets & intersection_lanelets,
    const std::optional<Polygon2d> & intersection_area);

  /**
   * @brief check collision
   */
  bool checkCollision(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, TargetObjects * target_objects,
    const PathLanelets & path_lanelets, const size_t closest_idx,
    const size_t last_intersection_stopline_candidate_idx, const double time_delay,
    const TrafficPrioritizedLevel & traffic_prioritized_level);

  /**
   * @brief return the CollisionKnowledge struct if the predicted path collides ego path spatially
   */
  std::optional<CollisionKnowledge> findPassageInterval(
    const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
    const autoware_auto_perception_msgs::msg::Shape & shape,
    const lanelet::BasicPolygon2d & ego_lane_poly,
    const std::optional<lanelet::ConstLanelet> & first_attention_lane_opt,
    const std::optional<lanelet::ConstLanelet> & second_attention_lane_opt);

  std::optional<size_t> checkAngleForTargetLanelets(
    const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
    const bool is_parked_vehicle) const;

  void cutPredictPathWithDuration(TargetObjects * target_objects, const double time_thr);
  void cutPredictPathWithinDuration(
    const builtin_interfaces::msg::Time & object_stamp, const double time_thr,
    autoware_auto_perception_msgs::msg::PredictedPath * predicted_path);

  /**
   * @fn
   * @brief calculate ego vehicle profile along the path inside the intersection as the sequence of
   * (time of arrival, traveled distance) from current ego position
   */
  TimeDistanceArray calcIntersectionPassingTime(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
    const size_t last_intersection_stopline_candidate_idx, const double time_delay,
    tier4_debug_msgs::msg::Float64MultiArrayStamped * debug_ttc_array);

  std::vector<lanelet::ConstLineString3d> generateDetectionLaneDivisions(
    lanelet::ConstLanelets detection_lanelets,
    const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution);

  /**
   * @fn
   * @brief check occlusion status
   */
  OcclusionType getOcclusionStatus(
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const lanelet::CompoundPolygon3d & first_attention_area,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const std::vector<lanelet::ConstLineString3d> & lane_divisions,
    const TargetObjects & target_objects);

  /*
  bool IntersectionModule::checkFrontVehicleDeceleration(
    lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
    const Polygon2d & stuck_vehicle_detect_area,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const double assumed_front_car_decel);
  */

  DebugData debug_data_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr ego_ttc_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr object_ttc_pub_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_INTERSECTION_HPP_
