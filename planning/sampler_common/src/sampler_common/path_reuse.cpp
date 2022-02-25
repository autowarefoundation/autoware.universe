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

#include "sampler_common/path_reuse.hpp"

#include "sampler_common/constraints/hard_constraint.hpp"

#include <boost/geometry/algorithms/distance.hpp>

#include <iostream>
#include <limits>

namespace sampler_common
{
bool tryToReusePath(
  const Path & path_to_reuse, const Point & current_pose, const double max_reuse_length,
  const double max_deviation, Path & reusable_path)
{
  // TODO(Maxime CLEMENT): use interpolation if we want better precision
  if (path_to_reuse.points.empty()) {
    return false;
  }

  reusable_path.clear();
  size_t start_index = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_to_reuse.points.size(); ++i) {
    const auto dist = boost::geometry::distance(current_pose, path_to_reuse.points[i]);
    if (dist < min_dist) {
      start_index = i;
      min_dist = dist;
    }
  }

  if (min_dist > max_deviation) {
    return false;
  }

  size_t end_index = start_index;
  double distance = 0.0;
  for (size_t i = start_index; i < path_to_reuse.points.size() - 2; ++i) {
    distance += path_to_reuse.intervals[i];
    if (distance > max_reuse_length) {
      end_index = i;
      break;
    }
  }

  auto copy = [&](const auto & from, auto & to) {
    to.insert(to.end(), std::next(from.begin(), start_index), std::next(from.begin(), end_index));
  };
  reusable_path.reserve(end_index);
  copy(path_to_reuse.points, reusable_path.points);
  copy(path_to_reuse.curvatures, reusable_path.curvatures);
  copy(path_to_reuse.intervals, reusable_path.intervals);
  // TODO(Maxime CLEMENT): jerk not yet computed
  // copy(path_to_reuse.jerks, reusable_path.jerks);
  copy(path_to_reuse.yaws, reusable_path.yaws);
  copy(path_to_reuse.intervals, reusable_path.intervals);

  std::vector<Path> paths = {reusable_path};

  // TODO(Maxime CLEMENT): points become empty when all paths are invalid but this should be
  // detected earlier
  return !reusable_path.points.empty();
}
}  // namespace sampler_common
