// Copyright 2024 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__DATA_STRUCTS_HPP_
#define BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__DATA_STRUCTS_HPP_

#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using tier4_planning_msgs::msg::LateralOffset;

struct RacingOvertakeParameters
{
  double too_close_to_overtake_distance;
  double start_overtake_distance;
  double prepare_overtake_distance;
  double back_to_center_start_distance;
  double back_to_center_end_distance;
  double ego_course_width;
};

}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__DATA_STRUCTS_HPP_
