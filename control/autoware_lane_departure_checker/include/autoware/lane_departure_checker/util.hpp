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

#ifndef AUTOWARE__LANE_DEPARTURE_CHECKER__UTIL_HPP_
#define AUTOWARE__LANE_DEPARTURE_CHECKER__UTIL_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

namespace autoware::lane_departure_checker
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length);
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__UTIL_HPP_
