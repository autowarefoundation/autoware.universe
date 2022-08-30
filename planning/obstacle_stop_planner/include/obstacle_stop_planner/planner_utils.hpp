
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

#ifndef OBSTACLE_STOP_PLANNER__PLANNER_UTILS_HPP_
#define OBSTACLE_STOP_PLANNER__PLANNER_UTILS_HPP_

#include "obstacle_stop_planner/planner_data.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

#include <string>
#include <utility>
#include <vector>

namespace motion_planning
{

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_autoware_utils::Point2d;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

bool validCheckDecelPlan(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE1)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @param (t_min) duration of constant deceleration [s]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType1(
  const double v0, const double vt, const double a0, const double am, const double ja,
  const double jd, const double t_min);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE2)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType2(
  const double v0, const double vt, const double a0, const double ja, const double jd);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE3)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType3(
  const double v0, const double vt, const double a0, const double ja);

boost::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec);

boost::optional<std::pair<double, double>> calcFeasibleMarginAndVelocity(
  const SlowDownParam & slow_down_param, const double dist_baselink_to_obstacle,
  const double current_vel, const double current_acc);

boost::optional<std::pair<size_t, TrajectoryPoint>> getForwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin);

boost::optional<std::pair<size_t, TrajectoryPoint>> getBackwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin);

boost::optional<std::pair<size_t, double>> findNearestFrontIndex(
  const size_t start_idx, const TrajectoryPoints & trajectory, const Point & point);

bool isInFrontOfTargetPoint(const Pose & pose, const Point & point);

bool checkValidIndex(const Pose & p_base, const Pose & p_next, const Pose & p_target);

std::string jsonDumpsPose(const Pose & pose);

DiagnosticStatus makeStopReasonDiag(const std::string stop_reason, const Pose & stop_pose);

TrajectoryPoint getBackwardPointFromBasePoint(
  const TrajectoryPoint & p_from, const TrajectoryPoint & p_to, const TrajectoryPoint & p_base,
  const double backward_length);

rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr);

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__PLANNER_UTILS_HPP_
