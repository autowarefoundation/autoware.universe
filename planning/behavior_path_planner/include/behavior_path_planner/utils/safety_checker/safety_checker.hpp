// Copyright 2021 Tier IV, Inc.
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
#include "behavior_path_planner/parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/optional.hpp>

#include <algorithm>
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
using tier4_autoware_utils::Polygon2d;

struct TargetObjectIndices
{
  std::vector<size_t> current_lane{};
  std::vector<size_t> target_lane{};
  std::vector<size_t> other_lane{};
};

struct CollisionCheckParams
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
  bool check_car{true};         // Check object car
  bool check_truck{true};       // Check object truck
  bool check_bus{true};         // Check object bus
  bool check_trailer{true};     // Check object trailer
  bool check_unknown{true};     // Check object unknown
  bool check_bicycle{true};     // Check object bicycle
  bool check_motorcycle{true};  // Check object motorbike
  bool check_pedestrian{true};  // Check object pedestrian

  // Buffer and thresholds
  double lateral_buffer{0.2};
  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.046};

  // Time window for collision checking
  double check_start_time{4.0};
  double check_end_time{10.0};

  // Stopping and acceleration parameters
  double stopping_time{1.0};
  double acc_till_target_velocity{1.0};
  double expected_front_deceleration{-1.0};
  double expected_rear_deceleration{-1.0};

  // Route and dynamic objects information
  PredictedObjects::ConstSharedPtr dynamic_objects;
  TargetObjectIndices & dynamic_objects_indices;
  RouteHandler route_handler;
  lanelet::ConstLanelets reference_lanelets{};
  lanelet::ConstLanelets target_lanelets{};

  // Vehicle dimensions
  double vehicle_width{1.83};

  // Debug marker publishing option
  bool publish_debug_marker{false};
};


enum class TRAJECTORY_TYPE {
  LINEAR = 0,
  SPLINE = 1,
};

// plannerdataには依存しない
// どう拡張するのか分からない？
// 拡張イメージ湧けばOK

class SafetyChecker
{
public:
  explicit SafetyChecker(
    const std::shared_ptr<BehaviorPathPlannerParameters> & common_parameters,
    const std::shared_ptr<PlannerData> & planner_data)
  : planner_data_{planner_data}, common_parameters_{common_parameters}
  {
    collision_check_data_ = createCollisionCheckData(common_parameters, planner_data);
  }

  /**
   * @brief Check path is safe against dynamic obstacles.
   * @details This function checks the safety of the path by checking the collision with the dynamic
   * obstacles.
   */
  void isPathSafe(const PathWithLaneId & path);

private:
  std::shared_ptr<BehaviorPathPlannerParameters> common_parameters_{};
  std::shared_ptr<const PlannerData> planner_data_{};
  std::shared_ptr<CollisionCheckData> collision_check_data_{};

  CollisionCheckData createCollisionCheckData();
  PredictedPath createPredictedPath() const;
  lanelet::ConstLanelets getBackwardLanelets() const;
  TargetObjectIndices filterObjectIndices() const;
  boost::optional<std::pair<Pose, Polygon2d>> getEgoExpectedPoseAndConvertToPolygon() const;
  bool isSafeInLaneletCollisionCheck() const;
  bool isObjectIndexIncluded() const;
  bool isTargetObjectFront() const;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_HPP_
