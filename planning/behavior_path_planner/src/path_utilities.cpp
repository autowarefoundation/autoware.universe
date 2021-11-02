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

#include "behavior_path_planner/path_utilities.hpp"

#include "behavior_path_planner/utilities.hpp"

#include <autoware_utils/autoware_utils.hpp>
#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <opencv2/opencv.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
namespace util
{
using autoware_perception_msgs::msg::PredictedPath;
using autoware_planning_msgs::msg::PathPointWithLaneId;
using autoware_planning_msgs::msg::PathWithLaneId;

/**
 * @brief calc path arclength on each points from start point to end point.
 */
std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, size_t start, size_t end, double offset)
{
  std::vector<double> out;

  double sum = offset;
  out.push_back(sum);

  start = std::max(start + 1, size_t{1});
  end = std::min(end, path.points.size());

  for (size_t i = start; i < end; ++i) {
    sum += autoware_utils::calcDistance2d(path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

/**
 * @brief calc path arclength from start point to end point.
 */
double calcPathArcLength(const PathWithLaneId & path, size_t start, size_t end)
{
  if (path.points.size() < 2) {
    return 0.0;
  }

  // swap
  bool is_negative_direction = false;
  if (start > end) {
    std::swap(start, end);
    is_negative_direction = true;
  }

  start = std::max(start, size_t{1});
  end = std::min(end, path.points.size());

  double sum = 0.0;
  for (size_t i = start; i < end; ++i) {
    sum += autoware_utils::calcDistance2d(path.points.at(i).point, path.points.at(i - 1).point);
  }

  return is_negative_direction ? -sum : sum;
}

/**
 * @brief resamplePathWithSpline
 */
PathWithLaneId resamplePathWithSpline(const PathWithLaneId & path, double interval)
{
  const auto base_points = calcPathArcLengthArray(path);
  const auto sampling_points = autoware_utils::arange(0.0, base_points.back(), interval);

  if (base_points.empty() || sampling_points.empty()) {
    return path;
  }

  std::vector<double> base_x, base_y, base_z;
  for (const auto & p : path.points) {
    const auto & pos = p.point.pose.position;
    base_x.push_back(pos.x);
    base_y.push_back(pos.y);
    base_z.push_back(pos.z);
  }

  const auto resampled_x = interpolation::slerp(base_points, base_x, sampling_points);
  const auto resampled_y = interpolation::slerp(base_points, base_y, sampling_points);
  const auto resampled_z = interpolation::slerp(base_points, base_z, sampling_points);

  PathWithLaneId resampled_path{};
  resampled_path.header = path.header;
  resampled_path.drivable_area = path.drivable_area;

  // For Point X, Y, Z
  for (size_t i = 0; i < sampling_points.size(); ++i) {
    PathPointWithLaneId p{};
    p.point.pose.position =
      autoware_utils::createPoint(resampled_x.at(i), resampled_y.at(i), resampled_z.at(i));
    resampled_path.points.push_back(p);
  }

  // For LaneIds, Type, Twist
  //
  // ------|----|----|----|----|----|----|-------> resampled
  //      [0]  [1]  [2]  [3]  [4]  [5]  [6]
  //
  // --------|---------------|----------|---------> base
  //        [0]             [1]        [2]
  //
  // resampled[0~3] = base[0]
  // resampled[4~5] = base[1]
  // resampled[6] = base[2]
  //
  size_t base_idx{0};
  for (size_t i = 0; i < resampled_path.points.size(); ++i) {
    while (base_idx < base_points.size() - 1 && sampling_points.at(i) > base_points.at(base_idx)) {
      ++base_idx;
    }
    size_t ref_idx = std::max(static_cast<int>(base_idx) - 1, 0);
    if (i == resampled_path.points.size() - 1) {
      ref_idx = base_points.size() - 1;  // for last point
    }
    auto & p = resampled_path.points.at(i);
    p.lane_ids = path.points.at(ref_idx).lane_ids;
    p.point.type = path.points.at(ref_idx).point.type;
    p.point.twist = path.points.at(ref_idx).point.twist;
  }

  // For Yaw
  for (size_t i = 0; i < resampled_path.points.size() - 1; ++i) {
    const auto & p0 = resampled_path.points.at(i).point.pose.position;
    const auto & p1 = resampled_path.points.at(i + 1).point.pose.position;
    const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    resampled_path.points.at(i).point.pose.orientation =
      autoware_utils::createQuaternionFromYaw(yaw);
  }
  if (resampled_path.points.size() > 2) {
    resampled_path.points.back().point.pose.orientation =
      resampled_path.points.at(resampled_path.points.size() - 2).point.pose.orientation;
  }

  return resampled_path;
}

Path toPath(const PathWithLaneId & input)
{
  Path output{};
  output.header = input.header;
  output.drivable_area = input.drivable_area;
  output.points.resize(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    output.points.at(i) = input.points.at(i).point;
  }
  return output;
}

size_t getIdxByArclength(const PathWithLaneId & path, const Point & origin, const double signed_arc)
{
  if (path.points.empty()) {
    throw std::runtime_error("[getIdxByArclength] path points must be > 0");
  }

  const auto closest_idx = autoware_utils::findNearestIndex(path.points, origin);

  using autoware_utils::calcDistance2d;
  double sum_length = 0.0;
  if (signed_arc >= 0.0) {
    for (size_t i = closest_idx; i < path.points.size() - 1; ++i) {
      const auto next_i = i + 1;
      sum_length += calcDistance2d(path.points.at(i), path.points.at(next_i));
      if (sum_length > signed_arc) {
        return next_i;
      }
    }
    return path.points.size() - 1;
  } else {
    for (size_t i = closest_idx; i > 0; --i) {
      const auto next_i = i - 1;
      sum_length -= calcDistance2d(path.points.at(i), path.points.at(next_i));
      if (sum_length < signed_arc) {
        return next_i;
      }
    }
    return 0;
  }
}

void clipPathLength(
  PathWithLaneId & path, const Point base_pos, const double forward, const double backward)
{
  if (path.points.size() < 3) {
    return;
  }

  const auto start_idx = util::getIdxByArclength(path, base_pos, -backward);
  const auto end_idx = util::getIdxByArclength(path, base_pos, forward);

  const std::vector<PathPointWithLaneId> clipped_points{
    path.points.begin() + start_idx, path.points.begin() + end_idx + 1};

  path.points = clipped_points;
}

}  // namespace util
}  // namespace behavior_path_planner
