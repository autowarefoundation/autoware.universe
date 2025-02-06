// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace
{
// for debug
std::string toStr(const geometry_msgs::msg::Point & p)
{
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + ")";
}
std::string toStr(const autoware::behavior_path_planner::ShiftLine & p)
{
  return "start point = " + toStr(p.start.position) + ", end point = " + toStr(p.end.position) +
         ", start idx = " + std::to_string(p.start_idx) +
         ", end idx = " + std::to_string(p.end_idx) +
         ", length = " + std::to_string(p.end_shift_length);
}
std::string toStr(const std::vector<double> & v)
{
  std::stringstream ss;
  for (const auto & p : v) {
    ss << p << ", ";
  }
  return ss.str();
}
}  // namespace

namespace autoware::behavior_path_planner
{

using autoware::motion_utils::findNearestIndex;
using autoware::motion_utils::insertOrientation;
using autoware::motion_utils::removeFirstInvalidOrientationPoints;
using autoware::motion_utils::removeOverlapPoints;

void PathShifter::setPath(const PathWithLaneId & path)
{
  reference_path_ = path;

  update_shift_lines_indices(shift_lines_);
  sort_shift_lines_along_path(shift_lines_);
}
void PathShifter::setVelocity(const double velocity)
{
  velocity_ = velocity;
}

void PathShifter::setLateralAccelerationLimit(const double lateral_acc)
{
  lateral_acc_limit_ = lateral_acc;
}

void PathShifter::setLongitudinalAcceleration(const double longitudinal_acc)
{
  longitudinal_acc_ = longitudinal_acc;
}

void PathShifter::addShiftLine(const ShiftLine & line)
{
  shift_lines_.push_back(line);

  update_shift_lines_indices(shift_lines_);
  sort_shift_lines_along_path(shift_lines_);
}

void PathShifter::setShiftLines(const std::vector<ShiftLine> & lines)
{
  shift_lines_ = lines;

  update_shift_lines_indices(shift_lines_);
  sort_shift_lines_along_path(shift_lines_);
}

bool PathShifter::generate(
  ShiftedPath * shifted_path, const bool offset_back, const SHIFT_TYPE type) const
{
  RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "PathShifter::generate start!");

  // Guard
  if (reference_path_.points.empty()) {
    RCLCPP_ERROR_STREAM(logger_, "reference path is empty.");
    return false;
  }

  shifted_path->path = reference_path_;
  shifted_path->shift_length.resize(reference_path_.points.size(), 0.0);

  if (shift_lines_.empty()) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, clock_, 3000, "shift_lines_ is empty. Return reference with base offset.");
    shift_base_length(shifted_path, base_offset_);
    return true;
  }

  for (const auto & shift_line : shift_lines_) {
    if (shift_line.end_idx < shift_line.start_idx) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, clock_, 3000, "Invalid indices: end_idx is less than start_idx");
      return false;
    }

    const auto idx_gap = shift_line.end_idx - shift_line.start_idx;
    if (idx_gap <= 1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, clock_, 3000,
        "shift start point and end point can't be adjoining "
        "Maybe shift length is too short?");
      return false;
    }
  }

  // Check if the shift points are sorted correctly
  if (!check_shift_lines_alignment(shift_lines_)) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to sort shift points..!!");
    return false;
  }

  if (shift_lines_.front().start_idx == 0) {
    // if offset is applied on front side, shifting from first point is no problem
    if (offset_back) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, clock_, 3000,
        "shift start point is at the edge of path. It could cause undesired result."
        " Maybe path is too short for backward?");
    }
  }

  // Calculate shifted path
  type == SHIFT_TYPE::SPLINE ? apply_spline_shifter(shifted_path, offset_back)
                             : apply_linear_shifter(shifted_path);

  shifted_path->path.points = removeOverlapPoints(shifted_path->path.points);
  // Use orientation before shift to remove points in reverse order
  // before setting wrong azimuth orientation
  removeFirstInvalidOrientationPoints(shifted_path->path.points);
  size_t previous_size{shifted_path->path.points.size()};
  do {
    previous_size = shifted_path->path.points.size();
    // Set the azimuth orientation to the next point at each point
    insertOrientation(shifted_path->path.points, true);
    // Use azimuth orientation to remove points in reverse order
    removeFirstInvalidOrientationPoints(shifted_path->path.points);
  } while (previous_size != shifted_path->path.points.size());

  // DEBUG
  RCLCPP_DEBUG_STREAM_THROTTLE(
    logger_, clock_, 3000,
    "PathShifter::generate end. shift_lines_.size = " << shift_lines_.size());

  return true;
}

void PathShifter::apply_linear_shifter(ShiftedPath * shifted_path) const
{
  const auto arclength_arr = utils::calcPathArcLengthArray(reference_path_);

  shift_base_length(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  // For all shift_lines_,
  for (const auto & shift_line : shift_lines_) {
    const auto current_shift = shifted_path->shift_length.at(shift_line.end_idx);
    const auto delta_shift = shift_line.end_shift_length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_line.end_idx) - arclength_arr.at(shift_line.start_idx), epsilon);

    // For all path.points,
    for (size_t i = 0; i < shifted_path->path.points.size(); ++i) {
      // Set shift length.
      double ith_shift_length = 0.0;
      if (i < shift_line.start_idx) {
        ith_shift_length = 0.0;
      } else if (i <= shift_line.end_idx) {
        auto dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_line.start_idx);
        ith_shift_length = (dist_from_start / shifting_arclength) * delta_shift;
      } else {
        ith_shift_length = delta_shift;
      }

      // Apply shifting.
      add_lateral_offset_on_index_point(shifted_path, ith_shift_length, i);
    }
  }
}

void PathShifter::apply_spline_shifter(ShiftedPath * shifted_path, const bool offset_back) const
{
  const auto arclength_arr = utils::calcPathArcLengthArray(reference_path_);

  shift_base_length(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  // For all shift_lines,
  for (const auto & shift_line : shift_lines_) {
    // calc delta shift at the sp.end_idx so that the sp.end_idx on the path will have
    // the desired shift length.
    const auto current_shift = shifted_path->shift_length.at(shift_line.end_idx);
    const auto delta_shift = shift_line.end_shift_length - current_shift;

    RCLCPP_DEBUG(
      logger_, "current_shift = %f, sp.length = %f", current_shift, shift_line.end_shift_length);

    if (std::abs(delta_shift) < 0.01) {
      RCLCPP_DEBUG(logger_, "delta shift is zero. skip for this shift point.");
    }

    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_line.end_idx) - arclength_arr.at(shift_line.start_idx), epsilon);

    // TODO(Watanabe) write docs.
    // These points are defined to achieve the constant-jerk shifting (see the header description).
    const auto [base_distance, base_length] =
      calc_base_lengths(shifting_arclength, delta_shift, offset_back);

    RCLCPP_DEBUG(
      logger_, "base_distance = %s, base_length = %s", toStr(base_distance).c_str(),
      toStr(base_length).c_str());

    std::vector<double> query_distance;
    std::vector<double> query_length;

    // For all path.points,
    // Note: start_idx is not included since shift = 0,
    //       end_idx is not included since shift is considered out of spline.
    for (size_t i = shift_line.start_idx + 1; i < shift_line.end_idx; ++i) {
      const double dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_line.start_idx);
      query_distance.push_back(dist_from_start);
    }
    if (!query_distance.empty()) {
      query_length = autoware::interpolation::spline(base_distance, base_length, query_distance);
    }

    // Apply shifting.
    {
      size_t i = shift_line.start_idx + 1;
      for (const auto & itr : query_length) {
        add_lateral_offset_on_index_point(shifted_path, itr, i);
        ++i;
      }
    }

    if (offset_back) {
      // Apply shifting after shift
      for (size_t i = shift_line.end_idx; i < shifted_path->path.points.size(); ++i) {
        add_lateral_offset_on_index_point(shifted_path, delta_shift, i);
      }
    } else {
      // Apply shifting before shift
      for (size_t i = 0; i < shift_line.start_idx + 1; ++i) {
        add_lateral_offset_on_index_point(shifted_path, query_length.front(), i);
      }
    }
  }
}

std::pair<std::vector<double>, std::vector<double>>
PathShifter::get_base_lengths_without_accel_limit(
  const double arclength, const double shift_length, const bool offset_back)
{
  const auto s = arclength;
  const auto l = shift_length;
  std::vector<double> base_lon = {0.0, 1.0 / 4.0 * s, 3.0 / 4.0 * s, s};
  std::vector<double> base_lat = {0.0, 1.0 / 12.0 * l, 11.0 / 12.0 * l, l};

  if (!offset_back) std::reverse(base_lat.begin(), base_lat.end());

  return std::pair{base_lon, base_lat};
}

std::pair<std::vector<double>, std::vector<double>>
PathShifter::get_base_lengths_without_accel_limit(
  const double arclength, const double shift_length, const double velocity,
  const double longitudinal_acc, const double total_time, const bool offset_back)
{
  const auto s = arclength;
  const auto l = shift_length;
  const auto v0 = velocity;
  const auto a = longitudinal_acc;
  const auto t = total_time / 4;

  const double s1 = std::min(v0 * t + 0.5 * a * t * t, s);
  const double v1 = v0 + a * t;
  const double s2 = std::min(s1 + 2 * v1 * t + 2 * a * t * t, s);
  std::vector<double> base_lon = {0.0, s1, s2, s};
  std::vector<double> base_lat = {0.0, 1.0 / 12.0 * l, 11.0 / 12.0 * l, l};

  if (!offset_back) std::reverse(base_lat.begin(), base_lat.end());

  return std::pair{base_lon, base_lat};
}

std::pair<std::vector<double>, std::vector<double>> PathShifter::calc_base_lengths(
  const double arclength, const double shift_length, const bool offset_back) const
{
  const auto v0 = std::abs(velocity_);

  // For longitudinal acceleration, we only consider positive side
  // negative acceleration (deceleration) is treated as 0.0
  const double acc_threshold = 0.0001;
  const auto & a = longitudinal_acc_ > acc_threshold ? longitudinal_acc_ : 0.0;

  if (v0 < 1.0e-5 && a < acc_threshold) {
    // no need to consider acceleration limit
    RCLCPP_DEBUG(logger_, "set velocity is zero. lateral acc limit is ignored");
    return get_base_lengths_without_accel_limit(arclength, shift_length, offset_back);
  }

  const auto S = arclength;
  const auto L = std::abs(shift_length);
  const auto T = a > acc_threshold ? (-v0 + std::sqrt(v0 * v0 + 2 * a * S)) / a : S / v0;
  const auto lateral_a_max = 8.0 * L / (T * T);

  if (lateral_a_max < lateral_acc_limit_) {
    // no need to consider acceleration limit
    RCLCPP_DEBUG_THROTTLE(
      logger_, clock_, 3000, "No need to consider lateral acc limit. max: %f, limit: %f",
      lateral_a_max, lateral_acc_limit_);
    return get_base_lengths_without_accel_limit(S, shift_length, v0, a, T, offset_back);
  }

  const auto tj = T / 2.0 - 2.0 * L / (lateral_acc_limit_ * T);
  const auto ta = 4.0 * L / (lateral_acc_limit_ * T) - T / 2.0;
  const auto lat_jerk =
    (2.0 * lateral_acc_limit_ * lateral_acc_limit_ * T) / (lateral_acc_limit_ * T * T - 4.0 * L);

  if (tj < 0.0 || ta < 0.0 || lat_jerk < 0.0 || tj / T < 0.1) {
    // no need to consider acceleration limit
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 3000,
      "Acc limit is too small to be applied. Tj: %f, Ta: %f, j: %f, a_max: %f, acc_limit: %f", tj,
      ta, lat_jerk, lateral_a_max, lateral_acc_limit_);
    return get_base_lengths_without_accel_limit(S, shift_length, offset_back);
  }

  const auto tj3 = tj * tj * tj;
  const auto ta2_tj = ta * ta * tj;
  const auto ta_tj2 = ta * tj * tj;

  const auto s1 = std::min(tj * v0 + 0.5 * a * tj * tj, S);
  const auto v1 = v0 + a * tj;

  const auto s2 = std::min(s1 + ta * v1 + 0.5 * a * ta * ta, S);
  const auto v2 = v1 + a * ta;

  const auto s3 = std::min(s2 + tj * v2 + 0.5 * a * tj * tj, S);  // = s4
  const auto v3 = v2 + a * tj;

  const auto s5 = std::min(s3 + tj * v3 + 0.5 * a * tj * tj, S);
  const auto v5 = v3 + a * tj;

  const auto s6 = std::min(s5 + ta * v5 + 0.5 * a * ta * ta, S);
  const auto v6 = v5 + a * ta;

  const auto s7 = std::min(s6 + tj * v6 + 0.5 * a * tj * tj, S);

  const auto sign = shift_length > 0.0 ? 1.0 : -1.0;
  const auto l1 = sign * (1.0 / 6.0 * lat_jerk * tj3);
  const auto l2 =
    sign * (1.0 / 6.0 * lat_jerk * tj3 + 0.5 * lat_jerk * ta_tj2 + 0.5 * lat_jerk * ta2_tj);
  const auto l3 =
    sign * (lat_jerk * tj3 + 1.5 * lat_jerk * ta_tj2 + 0.5 * lat_jerk * ta2_tj);  // = l4
  const auto l5 =
    sign * (11.0 / 6.0 * lat_jerk * tj3 + 2.5 * lat_jerk * ta_tj2 + 0.5 * lat_jerk * ta2_tj);
  const auto l6 =
    sign * (11.0 / 6.0 * lat_jerk * tj3 + 3.0 * lat_jerk * ta_tj2 + lat_jerk * ta2_tj);
  const auto l7 = sign * (2.0 * lat_jerk * tj3 + 3.0 * lat_jerk * ta_tj2 + lat_jerk * ta2_tj);

  std::vector<double> base_lon = {0.0, s1, s2, s3, s5, s6, s7};
  std::vector<double> base_lat = {0.0, l1, l2, l3, l5, l6, l7};
  if (!offset_back) std::reverse(base_lat.begin(), base_lat.end());

  return {base_lon, base_lat};
}

void PathShifter::update_shift_lines_indices(ShiftLineArray & shift_lines) const
{
  if (reference_path_.points.empty()) {
    RCLCPP_ERROR(
      logger_, "reference path is empty, setPath is needed before addShiftLine/setShiftLines.");
  }

  for (auto & l : shift_lines) {
    // TODO(murooka) remove findNearestIndex for except
    // lane_following to support u-turn & crossing path
    l.start_idx = findNearestIndex(reference_path_.points, l.start.position);
    // TODO(murooka) remove findNearestIndex for except
    // lane_following to support u-turn & crossing path
    l.end_idx = findNearestIndex(reference_path_.points, l.end.position);
  }
}

bool PathShifter::check_shift_lines_alignment(const ShiftLineArray & shift_lines) const
{
  for (const auto & l : shift_lines) {
    RCLCPP_DEBUG(logger_, "shift point = %s", toStr(l).c_str());
  }

  for (const auto & l : shift_lines) {
    if (l.start_idx > l.end_idx) {
      RCLCPP_ERROR(logger_, "shift_line must satisfy start_idx <= end_idx.");
      return false;
    }
  }

  return true;
}

void PathShifter::sort_shift_lines_along_path(ShiftLineArray & shift_lines) const
{
  if (shift_lines.empty()) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "shift_lines is empty. do nothing.");
    return;
  }

  const auto unsorted_shift_lines = shift_lines;

  // Calc indices sorted by "shift start point index" order
  std::vector<size_t> sorted_indices(unsorted_shift_lines.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(), [&](size_t i, size_t j) {
    return unsorted_shift_lines.at(i).start_idx < unsorted_shift_lines.at(j).start_idx;
  });

  // Set shift points and index by sorted_indices
  ShiftLineArray sorted_shift_lines;
  for (const auto sorted_i : sorted_indices) {
    sorted_shift_lines.push_back(unsorted_shift_lines.at(sorted_i));
  }

  shift_lines = sorted_shift_lines;

  // Debug
  for (const auto & l : unsorted_shift_lines) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "unsorted_shift_lines: " << toStr(l));
  }
  for (const auto & i : sorted_indices) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "sorted_indices i = " << i);
  }
  for (const auto & l : sorted_shift_lines) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, clock_, 3000, "sorted_shift_lines: in order: " << toStr(l));
  }
  RCLCPP_DEBUG(logger_, "PathShifter::sortShiftLinesAlongPath end.");
}

void PathShifter::removeBehindShiftLineAndSetBaseOffset(const size_t nearest_idx)
{
  // If shift_line.end is behind the ego_pose, remove the shift_line and
  // set its shift_length to the base_offset.
  ShiftLineArray new_shift_lines;
  ShiftLineArray removed_shift_lines;
  for (const auto & sl : shift_lines_) {
    (sl.end_idx > nearest_idx) ? new_shift_lines.push_back(sl) : removed_shift_lines.push_back(sl);
  }

  double new_base_offset = base_offset_;
  if (!removed_shift_lines.empty()) {
    const auto last_removed_sl = std::max_element(
      removed_shift_lines.begin(), removed_shift_lines.end(),
      [](const auto & a, const auto & b) { return a.end_idx > b.end_idx; });
    new_base_offset = last_removed_sl->end_shift_length;
  }

  // remove accumulated floating noise
  if (std::abs(new_base_offset) < 1.0e-4) {
    new_base_offset = 0.0;
  }

  RCLCPP_DEBUG(
    logger_, "shift_lines size: %lu -> %lu", shift_lines_.size(), new_shift_lines.size());

  setShiftLines(new_shift_lines);

  set_base_offset(new_base_offset);
}

void PathShifter::add_lateral_offset_on_index_point(ShiftedPath * path, double offset, size_t index)
{
  if (fabs(offset) < 1.0e-8) {
    return;
  }

  auto & p = path->path.points.at(index).point.pose;
  double yaw = tf2::getYaw(p.orientation);
  p.position.x -= std::sin(yaw) * offset;
  p.position.y += std::cos(yaw) * offset;

  path->shift_length.at(index) += offset;
}

void PathShifter::shift_base_length(ShiftedPath * path, double offset)
{
  constexpr double base_offset_thr = 1.0e-4;
  if (std::abs(offset) > base_offset_thr) {
    for (size_t i = 0; i < path->path.points.size(); ++i) {
      add_lateral_offset_on_index_point(path, offset, i);
    }
  }
}

double PathShifter::getLastShiftLength() const
{
  if (shift_lines_.empty()) {
    return base_offset_;
  }

  const auto furthest = std::max_element(
    shift_lines_.begin(), shift_lines_.end(),
    [](const auto & a, const auto & b) { return a.end_idx < b.end_idx; });

  return furthest->end_shift_length;
}

std::optional<ShiftLine> PathShifter::getLastShiftLine() const
{
  if (shift_lines_.empty()) {
    return {};
  }

  const auto furthest = std::max_element(
    shift_lines_.begin(), shift_lines_.end(),
    [](const auto & a, const auto & b) { return a.end_idx > b.end_idx; });

  return *furthest;
}

}  // namespace autoware::behavior_path_planner
