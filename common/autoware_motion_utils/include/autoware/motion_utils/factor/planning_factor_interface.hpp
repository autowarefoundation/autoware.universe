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

#ifndef AUTOWARE__MOTION_UTILS__FACTOR__PLANNING_FACTOR_INTERFACE_HPP_
#define AUTOWARE__MOTION_UTILS__FACTOR__PLANNING_FACTOR_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/control_point.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/planning_factor.hpp>
#include <tier4_planning_msgs/msg/planning_factor_array.hpp>
#include <tier4_planning_msgs/msg/safety_factor_array.hpp>

#include <string>
#include <vector>

namespace autoware::motion_utils
{

using geometry_msgs::msg::Pose;
using tier4_planning_msgs::msg::ControlPoint;
using tier4_planning_msgs::msg::PlanningFactor;
using tier4_planning_msgs::msg::PlanningFactorArray;
using tier4_planning_msgs::msg::SafetyFactorArray;

class PlanningFactorInterface
{
public:
  PlanningFactorInterface(rclcpp::Node * node, const std::string & name);

  /**
   * @brief factor setter for single control point.
   *
   * @param path points.
   * @param ego current pose.
   * @param control point pose. (e.g. stop or slow down point pose)
   * @param behavior of this planning factor.
   * @param safety factor.
   * @param driving direction.
   * @param target velocity of the control point.
   * @param shift length of the control point.
   * @param detail information.
   */
  template <class PointType>
  void add(
    const std::vector<PointType> & points, const Pose & ego_pose, const Pose & control_point_pose,
    const uint16_t behavior, const SafetyFactorArray & safety_factors,
    const bool is_driving_forward = true, const double velocity = 0.0,
    const double shift_length = 0.0, const std::string & detail = "");

  /**
   * @brief factor setter for two control points (section).
   *
   * @param path points.
   * @param ego current pose.
   * @param control section start pose. (e.g. lane change start point pose)
   * @param control section end pose. (e.g. lane change end point pose)
   * @param behavior of this planning factor.
   * @param safety factor.
   * @param driving direction.
   * @param target velocity of the control point.
   * @param shift length of the control point.
   * @param detail information.
   */
  template <class PointType>
  void add(
    const std::vector<PointType> & points, const Pose & ego_pose, const Pose & start_pose,
    const Pose & end_pose, const uint16_t behavior, const SafetyFactorArray & safety_factors,
    const bool is_driving_forward = true, const double velocity = 0.0,
    const double shift_length = 0.0, const std::string & detail = "");

  /**
   * @brief factor setter for single control point.
   *
   * @param distance to control point.
   * @param control point pose. (e.g. stop point pose)
   * @param behavior of this planning factor.
   * @param safety factor.
   * @param driving direction.
   * @param target velocity of the control point.
   * @param shift length of the control point.
   * @param detail information.
   */
  void add(
    const double distance, const Pose & control_point_pose, const uint16_t behavior,
    const SafetyFactorArray & safety_factors, const bool is_driving_forward = true,
    const double velocity = 0.0, const double shift_length = 0.0, const std::string & detail = "");

  /**
   * @brief factor setter for two control points (section).
   *
   * @param distance to control section start point.
   * @param distance to control section end point.
   * @param control section start pose. (e.g. lane change start point pose)
   * @param control section end pose. (e.g. lane change end point pose)
   * @param behavior of this planning factor.
   * @param safety factor.
   * @param driving direction.
   * @param target velocity of the control point.
   * @param shift length of the control point.
   * @param detail information.
   */
  void add(
    const double start_distance, const double end_distance, const Pose & start_pose,
    const Pose & end_pose, const uint16_t behavior, const SafetyFactorArray & safety_factors,
    const bool is_driving_forward = true, const double velocity = 0.0,
    const double shift_length = 0.0, const std::string & detail = "");

  /**
   * @brief publish planning factors.
   */
  void publish();

private:
  std::string name_;

  rclcpp::Publisher<PlanningFactorArray>::SharedPtr pub_factors_;

  rclcpp::Clock::SharedPtr clock_;

  std::vector<PlanningFactor> factors_;
};

extern template void PlanningFactorInterface::add<tier4_planning_msgs::msg::PathPointWithLaneId>(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> &, const Pose &, const Pose &,
  const uint16_t behavior, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::msg::PathPoint>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &, const Pose &, const Pose &,
  const uint16_t behavior, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::msg::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &, const Pose &, const Pose &,
  const uint16_t behavior, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);

extern template void PlanningFactorInterface::add<tier4_planning_msgs::msg::PathPointWithLaneId>(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> &, const Pose &, const Pose &,
  const Pose &, const uint16_t behavior, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::msg::PathPoint>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &, const Pose &, const Pose &,
  const Pose &, const uint16_t behavior, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::msg::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &, const Pose &, const Pose &,
  const Pose &, const uint16_t behavior, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__FACTOR__PLANNING_FACTOR_INTERFACE_HPP_
