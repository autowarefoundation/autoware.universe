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

#include "autoware/trajectory/utils/shift.hpp"

#include "autoware/trajectory/detail/logging.hpp"
#include "autoware/trajectory/interpolator/cubic_spline.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autoware::trajectory::detail::impl
{

// This function calculates base longitudinal and lateral lengths
// when acceleration limit is not considered (simple division approach).
std::pair<std::vector<double>, std::vector<double>> get_base_lengths_without_accel_limit(
  const double arc_length, const double shift_length)
{
  // Alias for clarity
  const double total_arc_length = arc_length;
  const double total_shift_length = shift_length;

  // Prepare base longitudinal positions
  const std::vector<double> base_longitudinal = {
    0.0, 0.25 * total_arc_length, 0.75 * total_arc_length, total_arc_length};

  // Prepare base lateral positions
  const std::vector<double> base_lateral = {
    0.0, 1.0 / 12.0 * total_shift_length, 11.0 / 12.0 * total_shift_length, total_shift_length};

  return {base_longitudinal, base_lateral};
}

// This function calculates base longitudinal and lateral lengths
// when acceleration limit is not considered, but velocity and acceleration are known.
std::pair<std::vector<double>, std::vector<double>> get_base_lengths_without_accel_limit(
  const double arc_length, const double shift_length, const double velocity,
  const double longitudinal_acc, const double total_time)
{
  // Aliases for clarity
  const double total_arc_length = arc_length;
  const double total_shift_length = shift_length;
  const double v0 = velocity;         // initial velocity
  const double a = longitudinal_acc;  // longitudinal acceleration
  const double t = total_time / 4.0;  // quarter of total time

  // Calculate first segment in longitudinal direction
  // s1 = v0 * t + 1/2 * a * t^2 (but capped by total_arc_length)
  const double segment_s1 = std::min(v0 * t + 0.5 * a * t * t, total_arc_length);

  // Velocity at the end of first segment
  const double v1 = v0 + a * t;

  // Calculate second segment in longitudinal direction
  // s2 = s1 + 2 * v1 * t + 2 * a * t^2 (but capped by total_arc_length)
  const double segment_s2 = std::min(segment_s1 + 2.0 * v1 * t + 2.0 * a * t * t, total_arc_length);

  // Prepare base longitudinal positions
  const std::vector<double> base_longitudinal = {0.0, segment_s1, segment_s2, total_arc_length};

  // Prepare base lateral positions (simple division approach as original)
  const std::vector<double> base_lateral = {
    0.0, 1.0 / 12.0 * total_shift_length, 11.0 / 12.0 * total_shift_length, total_shift_length};

  return {base_longitudinal, base_lateral};
}
std::pair<std::vector<double>, std::vector<double>> calc_base_lengths(
  const double & arc_length, const double & shift_length, const ShiftParameters & shift_parameters)
{
  // Threshold for treating acceleration as zero
  const double acc_threshold = 1.0e-4;

  // Extract initial velocity and clamp negative acceleration to zero
  const double initial_vel = std::abs(shift_parameters.velocity);
  const double used_lon_acc =
    (shift_parameters.longitudinal_acc > acc_threshold) ? shift_parameters.longitudinal_acc : 0.0;

  // If there is no need to consider acceleration limit
  if (initial_vel < 1.0e-5 && used_lon_acc < acc_threshold) {
    RCLCPP_DEBUG(
      get_logger(),
      "Velocity is effectively zero. "
      "No lateral acceleration limit will be applied.");
    return get_base_lengths_without_accel_limit(arc_length, shift_length);
  }

  // Prepare main parameters
  const double target_arclength = arc_length;
  const double target_shift_abs = std::abs(shift_length);

  // Calculate total time (total_time) to travel 'target_arclength'
  // If used_lon_acc is valid (> 0), use time from kinematic formula. Otherwise, use s/v
  const double total_time =
    (used_lon_acc > acc_threshold)
      ? (-initial_vel +
         std::sqrt(initial_vel * initial_vel + 2.0 * used_lon_acc * target_arclength)) /
          used_lon_acc
      : (target_arclength / initial_vel);

  // Calculate the maximum lateral acceleration if we do not add further constraints
  const double max_lateral_acc = 8.0 * target_shift_abs / (total_time * total_time);

  // If the max_lateral_acc is already below the limit, no need to reduce it
  if (max_lateral_acc < shift_parameters.lateral_acc_limit) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), get_clock(), 3000, "No need to consider lateral acc limit. max: %f, limit: %f",
      max_lateral_acc, shift_parameters.lateral_acc_limit);
    return get_base_lengths_without_accel_limit(
      target_arclength, shift_length, initial_vel, used_lon_acc, total_time);
  }

  // Compute intermediate times (jerk_time / accel_time) and lateral jerk
  const double jerk_time =
    total_time / 2.0 - 2.0 * target_shift_abs / (shift_parameters.lateral_acc_limit * total_time);
  const double accel_time =
    4.0 * target_shift_abs / (shift_parameters.lateral_acc_limit * total_time) - total_time / 2.0;
  const double lateral_jerk =
    (2.0 * shift_parameters.lateral_acc_limit * shift_parameters.lateral_acc_limit * total_time) /
    (shift_parameters.lateral_acc_limit * total_time * total_time - 4.0 * target_shift_abs);

  // If computed times or jerk are invalid (negative or too small), skip the acc limit
  if (jerk_time < 0.0 || accel_time < 0.0 || lateral_jerk < 0.0 || (jerk_time / total_time) < 0.1) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), get_clock(), 3000,
      "The specified lateral acceleration limit appears too restrictive. "
      "No feasible jerk_time or accel_time was found. "
      "Possible reasons: (1) shift_length is too large, (2) lateral_acc_limit is too low, "
      "or (3) system kinematics are outside valid range.\n"
      "Details:\n"
      "  - jerk_time: %.4f\n"
      "  - accel_time: %.4f\n"
      "  - lateral_jerk: %.4f\n"
      "  - computed_lateral_acc: %.4f\n"
      "  - lateral_acc_limit: %.4f\n\n"
      "Suggestions:\n"
      "  - Increase the lateral_acc_limit.\n"
      "  - Decrease shift_length if possible.\n"
      "  - Re-check velocity and acceleration settings for consistency.",
      jerk_time, accel_time, lateral_jerk, max_lateral_acc, shift_parameters.lateral_acc_limit);
    return get_base_lengths_without_accel_limit(target_arclength, shift_length);
  }

  // Precompute powers for jerk_time and accel_time
  const double jerk_time3 = jerk_time * jerk_time * jerk_time;
  const double accel_time2_jerk = accel_time * accel_time * jerk_time;
  const double accel_time_jerk2 = accel_time * jerk_time * jerk_time;

  // ------------------------------------------------------
  // Calculate longitudinal base points
  // ------------------------------------------------------
  // Segment s1
  const double segment_s1 = std::min(
    jerk_time * initial_vel + 0.5 * used_lon_acc * jerk_time * jerk_time, target_arclength);
  const double v1 = initial_vel + used_lon_acc * jerk_time;

  // Segment s2
  const double segment_s2 = std::min(
    segment_s1 + accel_time * v1 + 0.5 * used_lon_acc * accel_time * accel_time, target_arclength);
  const double v2 = v1 + used_lon_acc * accel_time;

  // Segment s3 = s4
  const double segment_s3 = std::min(
    segment_s2 + jerk_time * v2 + 0.5 * used_lon_acc * jerk_time * jerk_time, target_arclength);
  const double v3 = v2 + used_lon_acc * jerk_time;

  // Segment s5
  const double segment_s5 = std::min(
    segment_s3 + jerk_time * v3 + 0.5 * used_lon_acc * jerk_time * jerk_time, target_arclength);
  const double v5 = v3 + used_lon_acc * jerk_time;

  // Segment s6
  const double segment_s6 = std::min(
    segment_s5 + accel_time * v5 + 0.5 * used_lon_acc * accel_time * accel_time, target_arclength);
  const double v6 = v5 + used_lon_acc * accel_time;

  // Segment s7
  const double segment_s7 = std::min(
    segment_s6 + jerk_time * v6 + 0.5 * used_lon_acc * jerk_time * jerk_time, target_arclength);

  // ------------------------------------------------------
  // Calculate lateral base points
  // ------------------------------------------------------
  // sign determines the direction of shift
  const double shift_sign = (shift_length > 0.0) ? 1.0 : -1.0;

  // Shift amounts at each segment
  const double shift_l1 = shift_sign * (1.0 / 6.0 * lateral_jerk * jerk_time3);
  const double shift_l2 =
    shift_sign * (1.0 / 6.0 * lateral_jerk * jerk_time3 + 0.5 * lateral_jerk * accel_time_jerk2 +
                  0.5 * lateral_jerk * accel_time2_jerk);
  const double shift_l3 =
    shift_sign * (lateral_jerk * jerk_time3 + 1.5 * lateral_jerk * accel_time_jerk2 +
                  0.5 * lateral_jerk * accel_time2_jerk);  // = shift_l4
  const double shift_l5 =
    shift_sign * (11.0 / 6.0 * lateral_jerk * jerk_time3 + 2.5 * lateral_jerk * accel_time_jerk2 +
                  0.5 * lateral_jerk * accel_time2_jerk);
  const double shift_l6 =
    shift_sign * (11.0 / 6.0 * lateral_jerk * jerk_time3 + 3.0 * lateral_jerk * accel_time_jerk2 +
                  lateral_jerk * accel_time2_jerk);
  const double shift_l7 =
    shift_sign * (2.0 * lateral_jerk * jerk_time3 + 3.0 * lateral_jerk * accel_time_jerk2 +
                  lateral_jerk * accel_time2_jerk);

  // Construct the output vectors
  const std::vector<double> base_lon = {0.0,        segment_s1, segment_s2, segment_s3,
                                        segment_s5, segment_s6, segment_s7};
  const std::vector<double> base_lat = {0.0,      shift_l1, shift_l2, shift_l3,
                                        shift_l5, shift_l6, shift_l7};

  return {base_lon, base_lat};
}

void shift_impl(
  const std::vector<double> & bases, std::vector<double> * shift_lengths,
  const ShiftInterval & shift_interval, const ShiftParameters & shift_parameters)
{
  // lateral shift
  if (shift_interval.end <= 0.0) {
    for (size_t i = 0; i < bases.size(); ++i) {
      shift_lengths->at(i) += shift_interval.lateral_offset;
    }
    return;
  }

  const double shift_arc_length = std::abs(shift_interval.end - shift_interval.start);
  const bool shift_direction = shift_interval.end > shift_interval.start;
  // Calculate base lengths
  auto [base_lon, base_lat] = calc_base_lengths(
    shift_arc_length,               //
    shift_interval.lateral_offset,  //
    shift_parameters);

  auto cubic_spline =
    interpolator::CubicSpline::Builder{}.set_bases(base_lon).set_values(base_lat).build();

  if (!cubic_spline) {
    throw std::runtime_error(
      "Failed to build cubic spline for shift calculation.");  // This Exception should not be
                                                               // thrown.
  }
  for (size_t i = 0; i < bases.size(); ++i) {
    // Calculate the shift length at the current base
    const double s = bases.at(i);
    if (shift_direction) {
      if (s < shift_interval.start) {
        continue;
      }
      if (s <= shift_interval.end) {
        shift_lengths->at(i) += cubic_spline->compute(s - shift_interval.start);
      } else {
        shift_lengths->at(i) += cubic_spline->compute(shift_arc_length);
      }
    } else {
      if (s > shift_interval.start) {
        continue;
      }
      if (s >= shift_interval.end) {
        shift_lengths->at(i) += cubic_spline->compute(shift_interval.start - s);
      } else {
        shift_lengths->at(i) += cubic_spline->compute(shift_arc_length);
      }
    }
  }
}

}  // namespace autoware::trajectory::detail::impl
