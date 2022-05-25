// Copyright 2022 Tier IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_PLANNER__PLANNER_INTERFACE_HPP_
#define OBSTACLE_VELOCITY_PLANNER__PLANNER_INTERFACE_HPP_

#include "obstacle_velocity_planner/common_structs.hpp"
#include "obstacle_velocity_planner/utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "tier4_planning_msgs/msg/velocity_limit.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <vector>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_planning_msgs::msg::Trajectory;
using tier4_planning_msgs::msg::VelocityLimit;

class PlannerInterface
{
public:
  PlannerInterface(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const vehicle_info_util::VehicleInfo & vehicle_info)
  : longitudinal_info_(longitudinal_info), vehicle_info_(vehicle_info)
  {
    {  // cruise obstacle type
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.unknown")) {
        cruise_obstacle_types_.push_back(ObjectClassification::UNKNOWN);
      }
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.car")) {
        cruise_obstacle_types_.push_back(ObjectClassification::CAR);
      }
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.truck")) {
        cruise_obstacle_types_.push_back(ObjectClassification::TRUCK);
      }
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.bus")) {
        cruise_obstacle_types_.push_back(ObjectClassification::BUS);
      }
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.trailer")) {
        cruise_obstacle_types_.push_back(ObjectClassification::TRAILER);
      }
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.motorcycle")) {
        cruise_obstacle_types_.push_back(ObjectClassification::MOTORCYCLE);
      }
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.bicycle")) {
        cruise_obstacle_types_.push_back(ObjectClassification::BICYCLE);
      }
      if (node.declare_parameter<bool>("common.cruise_obstacle_type.pedestrian")) {
        cruise_obstacle_types_.push_back(ObjectClassification::PEDESTRIAN);
      }
    }

    {  // stop obstacle type
      if (node.declare_parameter<bool>("common.stop_obstacle_type.unknown")) {
        stop_obstacle_types_.push_back(ObjectClassification::UNKNOWN);
      }
      if (node.declare_parameter<bool>("common.stop_obstacle_type.car")) {
        stop_obstacle_types_.push_back(ObjectClassification::CAR);
      }
      if (node.declare_parameter<bool>("common.stop_obstacle_type.truck")) {
        stop_obstacle_types_.push_back(ObjectClassification::TRUCK);
      }
      if (node.declare_parameter<bool>("common.stop_obstacle_type.bus")) {
        stop_obstacle_types_.push_back(ObjectClassification::BUS);
      }
      if (node.declare_parameter<bool>("common.stop_obstacle_type.trailer")) {
        stop_obstacle_types_.push_back(ObjectClassification::TRAILER);
      }
      if (node.declare_parameter<bool>("common.stop_obstacle_type.motorcycle")) {
        stop_obstacle_types_.push_back(ObjectClassification::MOTORCYCLE);
      }
      if (node.declare_parameter<bool>("common.stop_obstacle_type.bicycle")) {
        stop_obstacle_types_.push_back(ObjectClassification::BICYCLE);
      }
      if (node.declare_parameter<bool>("common.stop_obstacle_type.pedestrian")) {
        stop_obstacle_types_.push_back(ObjectClassification::PEDESTRIAN);
      }
    }
  }

  PlannerInterface() = default;

  void setParams(
    const bool is_showing_debug_info, const double min_behavior_stop_margin,
    const double max_nearest_dist_deviation, const double max_nearest_yaw_deviation)
  {
    is_showing_debug_info_ = is_showing_debug_info;
    min_behavior_stop_margin_ = min_behavior_stop_margin;
    max_nearest_dist_deviation_ = max_nearest_dist_deviation;
    max_nearest_yaw_deviation_ = max_nearest_yaw_deviation;
  }

  /*
  // two kinds of velocity planning is supported.
  // 1. getZeroVelocityIndexWithVelocityLimit
  //   returns zero velocity index and velocity limit
  // 2. generateTrajectory
  //   returns trajectory with planned velocity
  virtual boost::optional<size_t> getZeroVelocityIndexWithVelocityLimit(
    [[maybe_unused]] const ObstacleVelocityPlannerData & planner_data,
    [[maybe_unused]] boost::optional<VelocityLimit> & vel_limit)
  {
    return {};
  };
  */

  virtual Trajectory generateTrajectory(
    const ObstacleVelocityPlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) = 0;

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    auto & i = longitudinal_info_;

    tier4_autoware_utils::updateParam<double>(parameters, "common.max_accel", i.max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.min_accel", i.min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.max_jerk", i.max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "common.min_jerk", i.min_jerk);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_object_accel", i.min_object_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.idling_time", i.idling_time);
  }

  virtual void updateParam([[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters) {}

  // TODO(shimizu) remove this function
  void setSmoothedTrajectory(const Trajectory::SharedPtr traj) { smoothed_trajectory_ptr_ = traj; }

  bool isCruiseObstacle(const uint8_t label)
  {
    const auto & types = cruise_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isStopObstacle(const uint8_t label)
  {
    const auto & types = stop_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

protected:
  // Parameters
  bool is_showing_debug_info_{false};
  LongitudinalInfo longitudinal_info_;
  double min_behavior_stop_margin_;
  double max_nearest_dist_deviation_;
  double max_nearest_yaw_deviation_;

  // Vehicle Parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  // TODO(shimizu) remove these parameters
  Trajectory::SharedPtr smoothed_trajectory_ptr_;

  double calcRSSDistance(
    const double ego_vel, const double obj_vel, const double margin = 0.0) const
  {
    const auto & i = longitudinal_info_;
    const double rss_dist_with_margin =
      ego_vel * i.idling_time + std::pow(ego_vel, 2) * 0.5 / std::abs(i.min_accel) -
      std::pow(obj_vel, 2) * 0.5 / std::abs(i.min_object_accel) + margin;
    return rss_dist_with_margin;
  }

private:
  std::vector<int> cruise_obstacle_types_;
  std::vector<int> stop_obstacle_types_;
};

#endif  // OBSTACLE_VELOCITY_PLANNER__PLANNER_INTERFACE_HPP_
