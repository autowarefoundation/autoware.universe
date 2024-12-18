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

#include <autoware/motion_utils/factor/planning_factor_interface.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <string>
#include <vector>

namespace autoware::motion_utils
{

PlanningFactorInterface::PlanningFactorInterface(rclcpp::Node * node, const std::string & name)
: name_{name},
  pub_factors_{
    node->create_publisher<PlanningFactorArray>("/planning/planning_factors/" + name, 1)},
  clock_{node->get_clock()}
{
}

template <class PointType>
void PlanningFactorInterface::add(
  const std::vector<PointType> & points, const Pose & ego_pose, const Pose & control_point_pose,
  const uint16_t behavior, const SafetyFactorArray & safety_factors, const bool is_driving_forward,
  const double velocity, const double shift_length, const std::string & detail)
{
  const auto distance = static_cast<float>(autoware::motion_utils::calcSignedArcLength(
    points, ego_pose.position, control_point_pose.position));
  add(
    distance, control_point_pose, behavior, safety_factors, is_driving_forward, velocity,
    shift_length, detail);
}

template void PlanningFactorInterface::add<tier4_planning_msgs::msg::PathPointWithLaneId>(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> &, const Pose &, const Pose &,
  const uint16_t behavior, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::msg::PathPoint>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &, const Pose &, const Pose &,
  const uint16_t behavior, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::msg::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &, const Pose &, const Pose &,
  const uint16_t behavior, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);

template <class PointType>
void PlanningFactorInterface::add(
  const std::vector<PointType> & points, const Pose & ego_pose, const Pose & start_pose,
  const Pose & end_pose, const uint16_t behavior, const SafetyFactorArray & safety_factors,
  const bool is_driving_forward, const double velocity, const double shift_length,
  const std::string & detail)
{
  const auto start_distance = static_cast<float>(
    autoware::motion_utils::calcSignedArcLength(points, ego_pose.position, start_pose.position));
  const auto end_distance = static_cast<float>(
    autoware::motion_utils::calcSignedArcLength(points, ego_pose.position, end_pose.position));
  add(
    start_distance, end_distance, start_pose, end_pose, behavior, safety_factors,
    is_driving_forward, velocity, shift_length, detail);
}

template void PlanningFactorInterface::add<tier4_planning_msgs::msg::PathPointWithLaneId>(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> &, const Pose &, const Pose &,
  const Pose &, const uint16_t behavior, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::msg::PathPoint>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &, const Pose &, const Pose &,
  const Pose &, const uint16_t behavior, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);
template void PlanningFactorInterface::add<autoware_planning_msgs::msg::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &, const Pose &, const Pose &,
  const Pose &, const uint16_t behavior, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);

void PlanningFactorInterface::add(
  const double distance, const Pose & control_point_pose, const uint16_t behavior,
  const SafetyFactorArray & safety_factors, const bool is_driving_forward, const double velocity,
  const double shift_length, const std::string & detail)
{
  const auto control_point = tier4_planning_msgs::build<ControlPoint>()
                               .pose(control_point_pose)
                               .velocity(velocity)
                               .shift_length(shift_length)
                               .distance(distance);

  const auto factor = tier4_planning_msgs::build<PlanningFactor>()
                        .module(name_)
                        .is_driving_forward(is_driving_forward)
                        .control_points({control_point})
                        .behavior(behavior)
                        .detail(detail)
                        .safety_factors(safety_factors);

  factors_.push_back(factor);
}

void PlanningFactorInterface::add(
  const double start_distance, const double end_distance, const Pose & start_pose,
  const Pose & end_pose, const uint16_t behavior, const SafetyFactorArray & safety_factors,
  const bool is_driving_forward, const double velocity, const double shift_length,
  const std::string & detail)
{
  const auto control_start_point = tier4_planning_msgs::build<ControlPoint>()
                                     .pose(start_pose)
                                     .velocity(velocity)
                                     .shift_length(shift_length)
                                     .distance(start_distance);

  const auto control_end_point = tier4_planning_msgs::build<ControlPoint>()
                                   .pose(end_pose)
                                   .velocity(velocity)
                                   .shift_length(shift_length)
                                   .distance(end_distance);

  const auto factor = tier4_planning_msgs::build<PlanningFactor>()
                        .module(name_)
                        .is_driving_forward(is_driving_forward)
                        .control_points({control_start_point, control_end_point})
                        .behavior(behavior)
                        .detail(detail)
                        .safety_factors(safety_factors);

  factors_.push_back(factor);
}

void PlanningFactorInterface::publish()
{
  PlanningFactorArray msg;
  msg.header.frame_id = "map";
  msg.header.stamp = clock_->now();
  msg.factors = factors_;

  pub_factors_->publish(msg);

  factors_.clear();
}

}  // namespace autoware::motion_utils
