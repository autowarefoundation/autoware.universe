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
  // trajectory generation
  double backward_lane_length{200.0};
  double prediction_time_resolution{0.5};

  bool use_predicted_path_outside_lanelet{false};
  bool use_all_predicted_path{false};

  // true by default
  bool check_car{true};         // check object car
  bool check_truck{true};       // check object truck
  bool check_bus{true};         // check object bus
  bool check_trailer{true};     // check object trailer
  bool check_unknown{true};     // check object unknown
  bool check_bicycle{true};     // check object bicycle
  bool check_motorcycle{true};  // check object motorbike
  bool check_pedestrian{true};  // check object pedestrian

  double lateral_buffer{0.2};

  // debug marker
  bool publish_debug_marker{false};

  double target_velocity double stopping_time double acc_till_target_velocity
    PredictedObjects::ConstSharedPtr dynamic_objects TargetObjectIndices & dynamic_objects_indices
      RouteHandler route_handler lanelet::ConstLanelets reference_lanelets{};
  lanelet::ConstLanelets target_lanelets{};
  double vehicle_width double forward_path_length

    double ego_nearest_dist_threshold double ego_nearest_yaw_threshold

    double check_end_time double check_start_time

    double expected_front_deceleration{-1.0};
  double expected_rear_deceleration
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
