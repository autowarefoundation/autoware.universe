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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_BASE_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_BASE_HPP_

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
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

struct TargetObjectIndices
{
  std::vector<size_t> current_lane{};
  std::vector<size_t> target_lane{};
  std::vector<size_t> other_lane{};
};

struct CollisionCheckData
{
  // trajectory generation
  double backward_lane_length{200.0};
  double prediction_time_resolution{0.5};
  // int longitudinal_acc_sampling_num{10};
  // int lateral_acc_sampling_num{10};

  // turn signal
  // double min_length_for_turn_signal_activation{10.0};

  // acceleration data
  // double min_longitudinal_acc{-1.0};
  // double max_longitudinal_acc{1.0};

  // collision check
  // bool enable_prepare_segment_collision_check{true};
  // double prepare_segment_ignore_object_velocity_thresh{0.1};
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
};

enum class TRAJECTORY_TYPE {
  LINEAR = 0,
  SPLINE = 1,
};

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

  ////////////////////////////////////////
  // Utility Functions
  ////////////////////////////////////////

protected:
  std::shared_ptr<BehaviorPathPlannerParameters> common_parameters_{};
  std::shared_ptr<const PlannerData> planner_data_{};
  std::shared_ptr<CollisionCheckData> collision_check_data_{};

private:
  virtual createCollisionCheckData() = 0;
  virtual PredictedPath createPredictedPath() const = 0;
  virtual getBackwardLanelets() const = 0;
  virtual filterObjectIndices() const = 0;
  virtual getEgoExpectedPoseAndConvertToPolygon() const = 0;
  virtual isSafeInLaneletCollisionCheck() const = 0;
  virtual isObjectIndexIncluded() const = 0;
  virtual isTargetObjectFront() const = 0;

  ////////////////////////////////////////
  // Helper Functions
  ////////////////////////////////////////

  /**
   * @brief Check if the shift points are aligned in order and have no conflict range.
   */
  bool checkShiftLinesAlignment(const ShiftLineArray & shift_lines) const;

  void addLateralOffsetOnIndexPoint(ShiftedPath * path, double offset, size_t index) const;

  void shiftBaseLength(ShiftedPath * path, double offset) const;

  void setBaseOffset(const double val)
  {
    RCLCPP_DEBUG(logger_, "base_offset is changed: %f -> %f", base_offset_, val);
    base_offset_ = val;
  }
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECKER__SAFETY_CHECKER_BASE_HPP_
