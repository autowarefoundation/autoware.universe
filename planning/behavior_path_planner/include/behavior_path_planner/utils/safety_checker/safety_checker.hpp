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
  double rear_vehicle_reaction_time{0.0};
  double rear_vehicle_safety_time_margin{0.0};
  double lateral_distance_max_threshold{0.0};
  double longitudinal_distance_min_threshold{0.0};
  double longitudinal_velocity_delta_time{0.0};
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

  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};

  // Stopping and acceleration parameters
  double stopping_time{1.0};
  double acc_till_target_velocity{1.0};
  double expected_front_deceleration{-1.0};
  double expected_rear_deceleration{-1.0};

  RSSparams rss_params{};

  // Vehicle dimensions
  vehicle_info_util::VehicleInfo vehicle_info{};
  double vehicle_width{1.83};

  // Debug marker publishing option
  bool publish_debug_marker{false};
};

struct SafetyCheckData
{
  // Route and dynamic objects information
  PredictedObjects::ConstSharedPtr dynamic_objects;
  TargetObjectIndices & dynamic_objects_indices;
  lanelet::ConstLanelets current_lanes{};
  lanelet::ConstLanelets target_lanes{};
  // lanelet::ConstLanelets reference_lanes{};

  marker_utils::CollisionCheckDebug collision_check_debug;
};

namespace safety_checker
{

class SafetyChecker
{
public:
  explicit SafetyChecker(
    const std::shared_ptr<SafetyCheckParams> & safety_check_params,
    const std::shared_ptr<SafetyCheckData> & safety_check_data)
  : safety_check_params_{safety_check_params}, safety_check_data_{safety_check_data}
  {
  }

  /**
   * @brief Check if a given path is safe based on the dynamic objects in the environment.
   *
   * @param path The path to be checked for safety. It is stored in the path_to_safety_check_
   * shared_ptr.
   * @param ego_odometry The odometry data for the ego vehicle. It is stored in the ego_odometry_
   * shared_ptr.
   *
   * @details The function first checks whether dynamic objects exist in the safety_check_data_.
   *          If no dynamic objects exist, the function assumes the path is safe and returns true.
   *          If the path to be checked is empty, the function assumes the path is unsafe and
   * returns false. If dynamic objects exist and the path is not empty, the function then checks if
   * the path is safe by calling the isSafeInLaneletCollisionCheck function.
   *
   * @return Returns true if the path is considered safe, false otherwise.
   */
  bool isPathSafe(const PathWithLaneId & path, const Odometry ego_odometry);

private:
  std::shared_ptr<SafetyCheckParams> safety_check_params_;
  std::shared_ptr<SafetyCheckData> safety_check_data_;
  std::shared_ptr<PathWithLaneId> path_to_safety_check_;
  std::shared_ptr<Odometry> ego_odometry_;

  /**
   * @brief Creates a predicted path based on the stored path and odometry, and safety check
   * parameters.
   *
   * @details This function uses the stored path and ego odometry, along with the safety check
   * parameters, to predict a future path for the ego vehicle. The parameters used in the prediction
   * include the current and target velocity, acceleration until the target velocity, the pose of
   * the ego vehicle, the resolution of the prediction time, and the stopping time.
   *
   *          The predicted path is created using the utility function
   * utils::createPredictedPathFromTargetVelocity, with the above parameters as inputs.
   *
   * @return Returns the predicted path for the ego vehicle.
   */
  PredictedPath createPredictedPath() const;

  /**
   * @brief Retrieves the sequence of backward lanelets based on ego vehicle's position and route
   * handler parameters.
   *
   * @details This function uses the stored ego odometry, safety check parameters and safety check
   * data to calculate the backward lanelets sequence from the ego vehicle's current position. The
   * length of the sequence is determined by the backward_lane_length parameter in safety check
   * parameters.
   *
   *          The function first checks if the target lanes are empty or if the length of the arc
   * from the ego vehicle to the target lanes is greater than or equal to the backward length. If
   * either of these conditions are met, the function returns an empty set of lanelets.
   *
   *          If the above conditions are not met, the function calculates the sequence of preceding
   * lanelets using the route handler's getPrecedingLaneletSequence function and appends them to the
   * backward_lanes set.
   *
   * @return Returns a set of lanelets constituting the backward sequence from the ego vehicle's
   * position.
   */
  lanelet::ConstLanelets getBackwardLanelets() const;

  /**
   * @brief Filters and categorizes dynamic objects based on their locations.
   *
   * @details This function uses safety check data and parameters, path to safety check and ego
   * odometry to filter dynamic objects in the current lane, target lane and others. The filtered
   * dynamic objects are those that fit into specific set limits and kinds of objects. The function
   * first determines the types of objects to check based on safety check parameters. Then it
   * creates basic polygons representing the current and target lanes. It checks each dynamic object
   * to see if it is of a type to check and if it intersects with these polygons.
   *
   *          If an object intersects with the current lane polygon, it is added to the current lane
   * objects. If an object intersects with the target lane polygon, it is added to the target lane
   * objects. If an object does not intersect with either the current lane or target lane polygons,
   * it is added to the others objects.
   *
   * @return Returns an object containing three lists of indices representing objects in the current
   * lane, target lane and others.
   */
  TargetObjectIndices filterObjectIndices() const;

  /**
   * @brief Calculates the expected pose of the ego vehicle and converts it to a polygon.
   *
   * @details This function generates a sequence of expected ego poses over time within a defined
   *          check_start_time and check_end_time interval and converts each pose to a polygon.
   *
   *          The function first creates the predicted path of the ego vehicle. Then for each
   * timestamp within the check_start_time to check_end_time interval (with step size equal to
   * time_resolution), it calculates the expected pose and converts it to a polygon.
   *
   *          All timestamps, poses, and polygons are stored in a PredictedPolygons structure.
   *
   * @return Returns a PredictedPolygons object containing the check durations, expected poses,
   *         and their corresponding polygons.
   */
  PredictedPolygons getEgoExpectedPoseAndConvertToPolygon() const;

  /**
   * @brief Checks if the ego vehicle's path is safe with respect to collision with dynamic objects.
   *
   * @details This function checks the safety of the ego vehicle's path by predicting the ego
   * vehicle's poses and comparing them with target objects' paths. It uses a utility function to
   * perform safety check for each target object. If any check returns unsafe, the function returns
   * false.
   *
   *          The function currently has placeholders for getting target objects and their paths,
   * which need to be implemented.
   *
   * @return Returns true if all safety checks pass (i.e., no collision is expected). Returns false
   * if a collision is expected with at least one target object.
   */
  bool isSafeInLaneletCollisionCheck() const;
  bool isObjectIndexIncluded(
    const size_t & index, const std::vector<size_t> & dynamic_objects_indices) const;
  bool isTargetObjectFront(const Polygon2d & obj_polygon) const;
};
}  // namespace safety_checker

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_HPP_
