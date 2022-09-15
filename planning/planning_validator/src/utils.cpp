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

#include "planning_validator/utils.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace planning_validator
{
using tier4_autoware_utils::calcCurvature;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getPoint;

std::pair<double, size_t> getMaxValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::max_element(v.begin(), v.end());
  const auto idx = std::distance(v.begin(), iter);
  return {*iter, idx};
}

std::pair<double, size_t> getMinValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::min_element(v.begin(), v.end());
  const auto idx = std::distance(v.begin(), iter);
  return {*iter, idx};
}

std::pair<double, size_t> getAbsMaxValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::max_element(
    v.begin(), v.end(), [](const auto & a, const auto & b) { return std::abs(a) < std::abs(b); });
  const auto idx = std::distance(v.begin(), iter);
  return {std::abs(*iter), idx};
}

// Do not interpolate.
Trajectory resampleTrajectory(const Trajectory & trajectory, const double min_interval)
{
  Trajectory resampled;
  resampled.header = trajectory.header;

  if (trajectory.points.empty()) {
    return resampled;
  }

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto prev = resampled.points.back();
    const auto curr = trajectory.points.at(i);
    if (calcDistance2d(prev, curr) > min_interval) {
      resampled.points.push_back(curr);
    }
  }
  return resampled;
}

std::vector<double> calcCurvature(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 3) {
    return std::vector<double>(trajectory.points.size(), 0.0);
  }

  std::vector<double> curvature_arr;
  curvature_arr.push_back(0.0);
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto p1 = getPoint(trajectory.points.at(i - 1));
    const auto p2 = getPoint(trajectory.points.at(i));
    const auto p3 = getPoint(trajectory.points.at(i + 1));
    try {
      curvature_arr.push_back(tier4_autoware_utils::calcCurvature(p1, p2, p3));
    } catch (...) {
      // maybe distance is too close
      curvature_arr.push_back(0.0);
    }
  }
  curvature_arr.push_back(0.0);
  return curvature_arr;
}

std::vector<double> calcIntervalDistance(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 2) {
    return std::vector<double>(trajectory.points.size(), 0.0);
  }

  std::vector<double> interval_distances;
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    interval_distances.push_back(
      calcDistance2d(trajectory.points.at(i), trajectory.points.at(i - 1)));
  }
  return interval_distances;
}

std::vector<double> calcLateralAcceleration(const Trajectory & trajectory)
{
  const auto curvatures = calcCurvature(trajectory);

  std::vector<double> lat_acc_arr;
  for (size_t i = 0; i < curvatures.size(); ++i) {
    const auto v = trajectory.points.at(i).longitudinal_velocity_mps;
    const auto lat_acc = v * v * curvatures.at(i);
    lat_acc_arr.push_back(lat_acc);
  }
  return lat_acc_arr;
}

std::vector<double> getLongitudinalAccArray(const Trajectory & trajectory)
{
  std::vector<double> acc_arr;
  for (const auto & p : trajectory.points) {
    acc_arr.push_back(p.acceleration_mps2);
  }
  return acc_arr;
}

std::vector<double> calcRelativeAngles(const Trajectory & trajectory)
{
  // We need at least three points to compute relative angle
  const size_t relative_angle_points_num = 3;
  if (trajectory.points.size() < relative_angle_points_num) {
    return {};
  }

  std::vector<double> relative_angles;

  for (size_t i = 0; i <= trajectory.points.size() - relative_angle_points_num; ++i) {
    const auto & p1 = trajectory.points.at(i).pose.position;
    const auto & p2 = trajectory.points.at(i + 1).pose.position;
    const auto & p3 = trajectory.points.at(i + 2).pose.position;

    const auto angle_a = tier4_autoware_utils::calcAzimuthAngle(p1, p2);
    const auto angle_b = tier4_autoware_utils::calcAzimuthAngle(p2, p3);

    // convert relative angle to [-pi ~ pi]
    const auto relative_angle = std::abs(tier4_autoware_utils::normalizeRadian(angle_b - angle_a));

    relative_angles.push_back(relative_angle);
  }

  return relative_angles;
}

std::vector<double> calcSteeringAngles(const Trajectory & trajectory, const double wheelbase)
{
  const auto curvature_to_steering = [](const auto k, const auto wheelbase) {
    return std::atan(k * wheelbase);
  };

  const auto curvatures = calcCurvature(trajectory);

  std::vector<double> steerings;
  for (const auto & k : curvatures) {
    steerings.push_back(curvature_to_steering(k, wheelbase));
  }
  return steerings;
}

std::vector<double> calcSteeringRates(const Trajectory & trajectory, const double wheelbase)
{
  const auto steerings = calcSteeringAngles(trajectory, wheelbase);

  std::vector<double> steering_rates;
  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto & p_prev = trajectory.points.at(i);
    const auto & p_next = trajectory.points.at(i + 1);
    const auto delta_s = calcDistance2d(p_prev, p_next);
    const auto v = 0.5 * (p_next.longitudinal_velocity_mps + p_prev.longitudinal_velocity_mps);
    const auto dt = delta_s / std::max(v, 1.0e-5);

    const auto steer_prev = steerings.at(i);
    const auto steer_next = steerings.at(i + 1);

    const auto steer_rate = (steer_next - steer_prev) / dt;
    steering_rates.push_back(steer_rate);
  }

  if (!steering_rates.empty()) {
    steering_rates.push_back(steering_rates.back());
  }

  return steering_rates;
}

bool checkFinite(const TrajectoryPoint & point)
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;

  using std::isfinite;
  const bool p_result = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
  const bool quat_result = isfinite(o.x) && isfinite(o.y) && isfinite(o.z) && isfinite(o.w);
  const bool v_result = isfinite(point.longitudinal_velocity_mps);
  const bool w_result = isfinite(point.heading_rate_rps);
  const bool a_result = isfinite(point.acceleration_mps2);

  return quat_result && p_result && v_result && w_result && a_result;
}

}  // namespace planning_validator
