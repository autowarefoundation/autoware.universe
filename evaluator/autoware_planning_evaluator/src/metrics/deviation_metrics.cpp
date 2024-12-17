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

#include "autoware/planning_evaluator/metrics/deviation_metrics.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/geometry/pose_deviation.hpp"

namespace planning_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

Accumulator<double> calcLateralDeviation(const Trajectory & ref, const Trajectory & traj)
{
  Accumulator<double> stat;

  if (ref.points.empty() || traj.points.empty()) {
    return stat;
  }

  /** TODO(Maxime CLEMENT):
   * need more precise calculation, e.g., lateral distance from spline of the reference traj
   */
  for (TrajectoryPoint p : traj.points) {
    const size_t nearest_index =
      autoware::motion_utils::findNearestIndex(ref.points, p.pose.position);
    stat.add(autoware::universe_utils::calcLateralDeviation(
      ref.points[nearest_index].pose, p.pose.position));
  }
  return stat;
}

Accumulator<double> calcNearestPoseDeviation(
  const Trajectory & prev_traj, const Trajectory & ref_traj, const Pose & ego_pose)
{
  Accumulator<double> stat;

  if (prev_traj.points.empty() || prev_traj.points.empty()) {
    return stat;
  }

  const auto prev_idx =
    autoware::motion_utils::findNearestIndex(prev_traj.points, ego_pose.position);
  const auto prev_p = prev_traj.points[prev_idx];
  const auto ref_idx = autoware::motion_utils::findNearestIndex(ref_traj.points, ego_pose.position);
  const auto ref_p = ref_traj.points[ref_idx];

  const double nearest_dist = autoware::universe_utils::calcDistance2d(prev_p, ref_p);

  stat.add(nearest_dist);

  return stat;
}

Accumulator<double> calcYawDeviation(const Trajectory & ref, const Trajectory & traj)
{
  Accumulator<double> stat;

  if (ref.points.empty() || traj.points.empty()) {
    return stat;
  }

  /** TODO(Maxime CLEMENT):
   * need more precise calculation, e.g., yaw distance from spline of the reference traj
   */
  for (TrajectoryPoint p : traj.points) {
    const size_t nearest_index =
      autoware::motion_utils::findNearestIndex(ref.points, p.pose.position);
    stat.add(autoware::universe_utils::calcYawDeviation(ref.points[nearest_index].pose, p.pose));
  }
  return stat;
}

Accumulator<double> calcVelocityDeviation(const Trajectory & ref, const Trajectory & traj)
{
  Accumulator<double> stat;

  if (ref.points.empty() || traj.points.empty()) {
    return stat;
  }

  // TODO(Maxime CLEMENT) need more precise calculation
  for (TrajectoryPoint p : traj.points) {
    const size_t nearest_index =
      autoware::motion_utils::findNearestIndex(ref.points, p.pose.position);
    stat.add(p.longitudinal_velocity_mps - ref.points[nearest_index].longitudinal_velocity_mps);
  }
  return stat;
}

Accumulator<double> calcLongitudinalDeviation(const Pose & base_pose, const Point & target_point)
{
  Accumulator<double> stat;
  stat.add(std::abs(autoware::universe_utils::calcLongitudinalDeviation(base_pose, target_point)));
  return stat;
}

Accumulator<double> calcLateralDeviation(const Pose & base_pose, const Point & target_point)
{
  Accumulator<double> stat;
  stat.add(std::abs(autoware::universe_utils::calcLateralDeviation(base_pose, target_point)));
  return stat;
}

Accumulator<double> calcYawDeviation(const Pose & base_pose, const Pose & target_pose)
{
  Accumulator<double> stat;
  stat.add(std::abs(autoware::universe_utils::calcYawDeviation(base_pose, target_pose)));
  return stat;
}
}  // namespace metrics
}  // namespace planning_diagnostics
