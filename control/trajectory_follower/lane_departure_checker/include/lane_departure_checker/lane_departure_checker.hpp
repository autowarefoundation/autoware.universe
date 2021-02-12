// Copyright 2020 Tier IV, Inc.
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

#ifndef LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_
#define LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_

#include <map>
#include <string>
#include <vector>

#include "boost/optional.hpp"

#include "lanelet2_core/LaneletMap.h"

#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"

#include "autoware_utils/geometry/boost_geometry.hpp"
#include "autoware_utils/geometry/pose_deviation.hpp"
#include "autoware_utils/ros/vehicle_info.hpp"

namespace lane_departure_checker
{
using autoware_utils::LinearRing2d;
using autoware_utils::PoseDeviation;

struct Param
{
  VehicleInfo vehicle_info;
  double footprint_margin;
  double resample_interval;
  double max_deceleration;
  double delay_time;
  double max_lateral_deviation;
  double max_longitudinal_deviation;
  double max_yaw_deviation_deg;
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_twist;
  lanelet::LaneletMapPtr lanelet_map;
  autoware_planning_msgs::msg::Route::ConstSharedPtr route;
  lanelet::ConstLanelets route_lanelets;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory;
};

struct Output
{
  Output()
  : resampled_trajectory(rosidl_runtime_cpp::MessageInitialization::ZERO) {}
  std::map<std::string, double> processing_time_map;
  bool will_leave_lane;
  bool is_out_of_lane;
  PoseDeviation trajectory_deviation;
  lanelet::ConstLanelets candidate_lanelets;
  autoware_planning_msgs::msg::Trajectory resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<LinearRing2d> vehicle_passing_areas;
};

class LaneDepartureChecker
{
public:
  Output update(const Input & input);

  void setParam(const Param & param) {param_ = param;}

private:
  Param param_;

  static PoseDeviation calcTrajectoryDeviation(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const geometry_msgs::msg::Pose & pose);

  //! This function assumes the input trajectory is sampled dense enough
  static autoware_planning_msgs::msg::Trajectory resampleTrajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const double interval);

  static autoware_planning_msgs::msg::Trajectory cutTrajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const double length);

  static std::vector<LinearRing2d> createVehicleFootprints(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const Param & param);

  static std::vector<LinearRing2d> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints);

  static LinearRing2d createHullFromFootprints(
    const LinearRing2d & area1, const LinearRing2d & area2);

  static bool willLeaveLane(
    const lanelet::ConstLanelets & candidate_lanelets,
    const std::vector<LinearRing2d> & vehicle_footprints);

  static bool isOutOfLane(
    const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint);
};
}  // namespace lane_departure_checker

#endif  // LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_
