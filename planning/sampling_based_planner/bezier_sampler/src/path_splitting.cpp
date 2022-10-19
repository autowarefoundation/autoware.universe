/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <bezier_sampler/path_splitting.hpp>

namespace bezier_sampler
{
// TODO(Maxime CLEMENT): curvatures should probably be estimated over longer distances than just
// successive points.
std::vector<std::pair<sampler_common::State, sampler_common::State>> splitPath(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path, double split_length,
  double max_length, int ego_pose_index)
{
  std::vector<std::pair<sampler_common::State, sampler_common::State>> splits;
  double total_arc_length(0.0);
  double sub_arc_length(0.0);
  sampler_common::State initial{};
  sampler_common::State final{};
  auto it = std::next(path.begin(), ego_pose_index);
  if (it != path.end() && std::next(it) != path.end()) {
    initial.pose = {it->pose.position.x, it->pose.position.y};
    // calculate heading using the angle towards the next point
    initial.heading = atan2(
      std::next(it)->pose.position.y - initial.pose.y(),
      std::next(it)->pose.position.x - initial.pose.x());
    initial.curvature = 0.0;
  }
  for (it = std::next(it); it != std::prev(path.end()); ++it) {
    if (sub_arc_length >= split_length || total_arc_length > max_length) {
      final.pose = {it->pose.position.x, it->pose.position.y};
      // calculate heading using the angle from the previous point
      final.heading = atan2(
        final.pose.y() - std::prev(it)->pose.position.y,
        final.pose.x() - std::prev(it)->pose.position.x);
      final.curvature = curvature(*std::prev(it), *it, *std::next(it));
      splits.emplace_back(initial, final);
      initial = final;
      sub_arc_length = 0.0;
      if (total_arc_length > max_length) break;
    } else {  // increment arc length
      const double arc_length = std::sqrt(
        (it->pose.position.x - std::prev(it)->pose.position.x) *
          (it->pose.position.x - std::prev(it)->pose.position.x) +
        (it->pose.position.y - std::prev(it)->pose.position.y) *
          (it->pose.position.y - std::prev(it)->pose.position.y));
      sub_arc_length += arc_length;
      total_arc_length += arc_length;
    }
  }
  return splits;
}
std::vector<std::pair<sampler_common::State, sampler_common::State>> splitPathByCurvature(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path, double split_curvature)
{
  std::vector<std::pair<sampler_common::State, sampler_common::State>> splits;
  double sum_curvature(0.0);
  sampler_common::State initial{};
  sampler_common::State final{};
  if (path.size() > 2) {
    initial.pose = {path[0].pose.position.x, path[0].pose.position.y};
    // calculate heading using the angle towards the next point
    initial.heading = std::atan2(
      path[1].pose.position.y - initial.pose.y(), path[1].pose.position.x - initial.pose.x());
    initial.curvature = curvature(path[0], path[1], path[2]);
  }
  for (auto it = std::next(path.begin()); it != std::prev(path.end()); ++it) {
    const double k = curvature(*std::prev(it), *it, *std::next(it));
    if (sum_curvature >= split_curvature) {
      final.pose = {it->pose.position.x, it->pose.position.y};
      // calculate heading using the angle from the previous point
      final.heading = std::atan2(
        final.pose.y() - std::prev(it)->pose.position.y,
        final.pose.x() - std::prev(it)->pose.position.x);
      final.curvature = k;  // TODO(Maxime CLEMENT): compute proper curvature
      splits.emplace_back(initial, final);
      initial = final;
      sum_curvature = 0.0;
    } else {  // increment curvature integral
      const double dist = std::sqrt(
        (it->pose.position.x - std::prev(it)->pose.position.x) *
          (it->pose.position.x - std::prev(it)->pose.position.x) +
        (it->pose.position.y - std::prev(it)->pose.position.y) *
          (it->pose.position.y - std::prev(it)->pose.position.y));

      sum_curvature += k * dist;
    }
  }
  std::cout << "Split by curvature size = " << splits.size() << std::endl;
  // TODO(Maxime CLEMENT): do we want to always include a subpath to the last point of the path ?
  return splits;
}
}  // namespace bezier_sampler
