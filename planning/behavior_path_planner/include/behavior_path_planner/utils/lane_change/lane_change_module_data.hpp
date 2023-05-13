// Copyright 2022 TIER IV, Inc.
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
#ifndef BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_

#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"
#include "interpolation/linear_interpolation.hpp"
#include "lanelet2_core/geometry/Lanelet.h"

#include "autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
struct LaneChangeParameters
{
  // trajectory generation
  double lane_change_finish_judge_buffer{3.0};
  double prediction_time_resolution{0.5};
  int lane_change_sampling_num{10};

  // collision check
  bool enable_prepare_segment_collision_check{true};
  double prepare_segment_ignore_object_velocity_thresh{0.1};
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

  // abort
  bool enable_cancel_lane_change{true};
  bool enable_abort_lane_change{false};

  double abort_delta_time{3.0};
  double abort_max_lateral_jerk{10.0};

  // debug marker
  bool publish_debug_marker{false};
};

enum class LaneChangeStates {
  Normal = 0,
  Cancel,
  Abort,
  Stop,
};

struct LaneChangePhaseInfo
{
  double prepare{0.0};
  double lane_changing{0.0};

  [[nodiscard]] double sum() const { return prepare + lane_changing; }
};

struct LaneChangeTargetObjectIndices
{
  std::vector<size_t> current_lane{};
  std::vector<size_t> target_lane{};
  std::vector<size_t> other_lane{};
};

enum class LaneChangeModuleType {
  NORMAL = 0,
  EXTERNAL_REQUEST,
  AVOIDANCE_BY_LANE_CHANGE,
};

struct AvoidanceByLCParameters
{
  std::shared_ptr<AvoidanceParameters> avoidance{};
  std::shared_ptr<LaneChangeParameters> lane_change{};

  // execute if the target object number is larger than this param.
  size_t execute_object_num{1};

  // execute only when the target object longitudinal distance is larger than this param.
  double execute_object_longitudinal_margin{0.0};

  // execute only when lane change end point is before the object.
  bool execute_only_when_lane_change_finish_before_object{false};
};

struct LateralAccelerationMap
{
  std::vector<double> base_vel;
  std::vector<double> base_min_acc;
  std::vector<double> base_max_acc;

  void add(const double velocity, const double min_acc, const double max_acc)
  {
    if (base_vel.size() != base_min_acc.size() || base_vel.size() != base_max_acc.size()) {
      return;
    }

    size_t idx = 0;
    for (size_t i = 0; i < base_vel.size(); ++i) {
      if (velocity < base_vel.at(i)) {
        break;
      }
      idx = i + 1;
    }

    base_vel.insert(base_vel.begin() + idx, velocity);
    base_min_acc.insert(base_min_acc.begin() + idx, min_acc);
    base_max_acc.insert(base_max_acc.begin() + idx, max_acc);
  }

  std::pair<double, double> find(const double velocity)
  {
    if (!base_vel.empty() && velocity < base_vel.front()) {
      return std::make_pair(base_min_acc.front(), base_max_acc.front());
    }
    if (!base_vel.empty() && velocity > base_vel.back()) {
      return std::make_pair(base_min_acc.back(), base_max_acc.back());
    }

    const double min_acc = interpolation::lerp(base_vel, base_min_acc, velocity);
    const double max_acc = interpolation::lerp(base_vel, base_max_acc, velocity);

    return std::make_pair(min_acc, max_acc);
  }
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_
