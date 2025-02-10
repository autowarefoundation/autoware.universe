// Copyright 2025 TIER IV, Inc.
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

#ifndef PID_BASED_PLANNER__CRUISE_PLANNING_DEBUG_INFO_HPP_
#define PID_BASED_PLANNER__CRUISE_PLANNING_DEBUG_INFO_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <array>

namespace autoware::motion_velocity_planner
{
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;

class CruisePlanningDebugInfo
{
public:
  enum class TYPE {
    // ego
    EGO_VELOCITY = 0,  // index: 0
    EGO_ACCELERATION,
    EGO_JERK,  // no data

    // cruise planning
    CRUISE_CURRENT_OBSTACLE_DISTANCE_RAW,  // index: 3
    CRUISE_CURRENT_OBSTACLE_DISTANCE_FILTERED,
    CRUISE_CURRENT_OBSTACLE_VELOCITY_RAW,
    CRUISE_CURRENT_OBSTACLE_VELOCITY_FILTERED,
    CRUISE_TARGET_OBSTACLE_DISTANCE,
    CRUISE_ERROR_DISTANCE_RAW,
    CRUISE_ERROR_DISTANCE_FILTERED,

    CRUISE_RELATIVE_EGO_VELOCITY,  // index: 10
    CRUISE_TIME_TO_COLLISION,

    CRUISE_TARGET_VELOCITY,  // index: 12
    CRUISE_TARGET_ACCELERATION,
    CRUISE_TARGET_JERK_RATIO,

    SIZE
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  static int getIndex(const TYPE type) { return static_cast<int>(type); }

  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<double, static_cast<int>(TYPE::SIZE)> get() const { return info_; }

  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void set(const TYPE type, const double val) { info_.at(static_cast<int>(type)) = val; }

  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void set(const int type, const double val) { info_.at(type) = val; }

  void reset() { info_.fill(0.0); }

  Float32MultiArrayStamped convert_to_message(const rclcpp::Time & current_time) const
  {
    Float32MultiArrayStamped msg{};
    msg.stamp = current_time;
    for (const auto & v : get()) {
      msg.data.push_back(v);
    }
    return msg;
  }

private:
  std::array<double, static_cast<int>(TYPE::SIZE)> info_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // PID_BASED_PLANNER__CRUISE_PLANNING_DEBUG_INFO_HPP_
