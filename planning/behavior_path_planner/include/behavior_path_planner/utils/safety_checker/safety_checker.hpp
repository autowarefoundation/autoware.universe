// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/marker_util/debug_utilities.hpp"
#include "behavior_path_planner/parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>
namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using tier4_autoware_utils::LineString2d;
using tier4_autoware_utils::Polygon2d;

struct TargetObjectIndices
{
  std::vector<size_t> current_lane{};
  std::vector<size_t> target_lane{};
  std::vector<size_t> other_lane{};
};

struct ObjectTypesToCheck
{
  bool check_car{true};
  bool check_truck{true};
  bool check_bus{true};
  bool check_trailer{true};
  bool check_unknown{true};
  bool check_bicycle{true};
  bool check_motorcycle{true};
  bool check_pedestrian{true};
};

struct PredictedPolygons
{
  std::vector<double> check_durations;
  std::vector<Pose> poses;
  std::vector<Polygon2d> polygons;
};

// struct PredictedPolygon
// {
//   double check_time;
//   Pose pose;
//   Polygon2d polygon;
// };

struct RSSparams
{
  double rear_vehicle_reaction_time{};
  double rear_vehicle_safety_time_margin{};
  double lateral_distance_max_threshold{};
  double longitudinal_distance_min_threshold{};
  double longitudinal_velocity_delta_time{};
};

struct SafetyCheckParams
{
  // Trajectory generation parameters
  double backward_lane_length{200.0};
  double prediction_time_resolution{0.5};
  double target_velocity{4.0};
  double forward_path_length{300.0};

  // Prediction options
  bool use_predicted_path_outside_lanelet{false};
  bool use_all_predicted_path{false};

  // Object types to check for collisions
  ObjectTypesToCheck object_types_to_check;

  // Buffer and thresholds
  double lateral_buffer{0.2};
  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.046};

  // Time window for collision checking
  double check_start_time{4.0};
  double check_end_time{10.0};

  lanelet::ConstLanelets current_lanes{};
  lanelet::ConstLanelets target_lanes{};
  // lanelet::ConstLanelets reference_lanes{};
  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};

  // Stopping and acceleration parameters
  double stopping_time{1.0};
  double acc_till_target_velocity{1.0};
  double expected_front_deceleration{-1.0};
  double expected_rear_deceleration{-1.0};

  RSSparams rss_params{};

  // Route and dynamic objects information
  PredictedObjects::ConstSharedPtr dynamic_objects;
  TargetObjectIndices & dynamic_objects_indices;
  lanelet::ConstLanelets reference_lanelets{};
  lanelet::ConstLanelets target_lanelets{};

  // Vehicle dimensions
  vehicle_info_util::VehicleInfo vehicle_info{};
  double vehicle_width{1.83};

  marker_utils::CollisionCheckDebug & collision_check_debug;

  // Debug marker publishing option
  bool publish_debug_marker{false};
};

enum class TRAJECTORY_TYPE {
  LINEAR = 0,
  SPLINE = 1,
};

namespace safety_checker
{

class SafetyChecker
{
public:
  explicit SafetyChecker(const std::shared_ptr<SafetyCheckParams> & safety_check_params)
  : safety_check_params_{safety_check_params}
  {
  }

  /**
   * @brief Check path is safe against dynamic obstacles.
   * @details This function checks the safety of the path by checking the collision with the dynamic
   * obstacles.
   */
  bool isPathSafe(const PathWithLaneId & path, const Odometry ego_odometry);

private:
  std::shared_ptr<SafetyCheckParams> safety_check_params_{};
  std::shared_ptr<PathWithLaneId> path_to_safety_check_{};
  std::shared_ptr<Odometry> ego_odometry_{};

  // TODO(Sugahara): remove const from function which change member variables
  PredictedPath createPredictedPath() const;
  lanelet::ConstLanelets getBackwardLanelets() const;
  TargetObjectIndices filterObjectIndices() const;
  PredictedPolygons getEgoExpectedPoseAndConvertToPolygon() const;
  bool isSafeInLaneletCollisionCheck() const;
  bool isObjectIndexIncluded(
    const size_t & index, const std::vector<size_t> & dynamic_objects_indices) const;
  bool isTargetObjectFront(const Polygon2d & obj_polygon) const;
};
}  // namespace safety_checker

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_HPP_
