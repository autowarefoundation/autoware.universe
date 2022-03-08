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

#include "sampler_node/path_generation.hpp"

#include "sampler_common/structures.hpp"

#include <bezier_sampler/bezier_sampling.hpp>
#include <frenet_planner/frenet_planner.hpp>
#include <sampler_common/path_reuse.hpp>
#include <sampler_node/prepare_inputs.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <iterator>
#include <numeric>
#include <vector>

namespace sampler_node
{
std::vector<sampler_common::Path> generateCandidatePaths(
  const sampler_common::State & initial_state, const sampler_common::Path & previous_path,
  const sampler_common::transform::Spline2D & path_spline,
  const autoware_auto_planning_msgs::msg::Path & path_msg,
  const sampler_common::Constraints & constraints)
{
  sampler_common::Path base_path;
  auto paths = generateFrenetPaths(initial_state, base_path, path_msg, path_spline, constraints);
  /*
  const auto bezier_paths = generateBezierPaths(initial_state, base_path, path_msg, path_spline);
  paths.insert(
    paths.end(), std::make_move_iterator(bezier_paths.begin()),
    std::make_move_iterator(bezier_paths.end()));
  */

  constexpr auto reuse_max_length_max = 50.0;  // TODO(Maxime CLEMENT): use length of previous_path
  const auto reuse_length_step = reuse_max_length_max / 4.0;
  constexpr auto reuse_max_deviation = 1.0;
  for (auto reuse_max_length = reuse_length_step; reuse_max_length <= reuse_max_length_max;
       reuse_max_length += reuse_length_step) {
    if (sampler_common::tryToReusePath(
          previous_path, initial_state.pose, reuse_max_length, reuse_max_deviation, constraints,
          base_path)) {
      const auto cost_mult = 1.0 - 0.3 * reuse_length_step / reuse_max_length_max;
      sampler_common::State end_of_reused_path;
      end_of_reused_path.pose = base_path.points.back();
      end_of_reused_path.heading = base_path.yaws.back();
      end_of_reused_path.curvature = base_path.curvatures.back();
      {
        const auto paths_from_prev_path =
          generateFrenetPaths(end_of_reused_path, base_path, path_msg, path_spline, constraints);
        for (const auto & path : paths_from_prev_path) {
          paths.push_back(base_path.extend(path));
          paths.back().cost *= cost_mult;
        }
      }
      /*
      {
        const auto paths_from_prev_path =
          generateBezierPaths(end_of_reused_path, base_path, path_msg, path_spline);
        for (const auto & path : paths_from_prev_path) {
          paths.push_back(base_path.extend(path));
          paths.back().cost *= cost_mult;
        }
      }
      */
    } else {
      // If we fail to reuse length L from the previous path, all lengths > L will also fail.
      break;
    }
  }
  return paths;
}
std::vector<sampler_common::Path> generateBezierPaths(
  const sampler_common::State & initial_state, const sampler_common::Path & base_path,
  const autoware_auto_planning_msgs::msg::Path & path_msg,
  const sampler_common::transform::Spline2D & path_spline)
{
  const auto base_path_length =
    std::accumulate(base_path.intervals.begin(), base_path.intervals.end(), 0.0);
  bezier_sampler::SamplingParameters sampling_params{};
  sampling_params.nb_k = 3;
  sampling_params.mk_min = 0.0;
  sampling_params.mk_max = 10.0;
  sampling_params.nb_t = 10;
  sampling_params.mt_min = 0.3;
  sampling_params.mt_max = 1.7;

  const double initial_s = path_spline.frenet(initial_state.pose).s;
  const double max_s =
    path_spline
      .frenet({path_msg.points.back().pose.position.x, path_msg.points.back().pose.position.y})
      .s;
  const auto target_lengths = {20, 30, 40};
  std::vector<sampler_common::Path> bezier_paths;
  for (const auto target_length : target_lengths) {
    if (target_length <= base_path_length) continue;
    const auto target_s = std::min(max_s, initial_s + target_length - base_path_length);
    if (target_s == max_s) break;
    sampler_common::State target_state{};
    target_state.pose = path_spline.cartesian({target_s, 0});
    target_state.curvature = path_spline.curvature(target_s);
    target_state.heading = path_spline.yaw(target_s);
    const auto beziers = bezier_sampler::sample(initial_state, target_state, sampling_params);

    constexpr double step = 0.01;
    for (const auto & bezier : beziers) {
      sampler_common::Path path;
      for (double t = 0.0; t <= 1.0; t += step) {
        const auto x_y = bezier.valueM(t);
        path.points.emplace_back(x_y[0], x_y[1]);
        path.yaws.emplace_back(bezier.heading(t));
        path.curvatures.push_back(bezier.curvature(t));
      }
      for (size_t i = 0; i + 1 < path.points.size(); ++i) {
        path.intervals.push_back(std::hypot(
          path.points[i + 1].x() - path.points[i].x(),
          path.points[i + 1].y() - path.points[i].y()));
      }
      bezier_paths.push_back(path);
    }
  }
  return bezier_paths;
}

std::vector<sampler_common::Path> generateFrenetPaths(
  const sampler_common::State & initial_state, const sampler_common::Path & base_path,
  const autoware_auto_planning_msgs::msg::Path & path,
  const sampler_common::transform::Spline2D & path_spline,
  const sampler_common::Constraints & constraints)
{
  (void)constraints;
  const auto sampling_parameters = prepareSamplingParameters(
    initial_state, path,
    std::accumulate(base_path.intervals.begin(), base_path.intervals.end(), 0.0), path_spline);

  frenet_planner::FrenetState initial_frenet_state;
  initial_frenet_state.position = path_spline.frenet(initial_state.pose);
  const auto s = initial_frenet_state.position.s;
  const auto d = initial_frenet_state.position.d;
  // Calculate Velocity and acceleration parametrized over arc length
  // From appendix I of Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
  const auto frenet_yaw = initial_state.heading - path_spline.yaw(s);
  const auto path_curv = path_spline.curvature(s);
  // TODO(Maxime CLEMENT): there should be an analytical way to get the derivative of curvature
  const auto delta_s = 0.001;
  initial_frenet_state.lateral_velocity = (1 - path_curv * d) * std::tan(frenet_yaw);
  const auto path_curv_deriv = (path_spline.curvature(s + delta_s) - path_curv) / delta_s;
  const auto cos_yaw = std::cos(frenet_yaw);
  if (cos_yaw == 0.0) {
    initial_frenet_state.lateral_acceleration = 0.0;
  } else {
    initial_frenet_state.lateral_acceleration =
      -(path_curv_deriv * d + path_curv * initial_frenet_state.lateral_velocity) *
        std::tan(frenet_yaw) +
      ((1 - path_curv * d) / (cos_yaw * cos_yaw)) *
        (initial_state.curvature * ((1 - path_curv * d) / cos_yaw) - path_curv);
  }
  return frenet_planner::generatePaths(path_spline, initial_frenet_state, sampling_parameters);
}
}  // namespace sampler_node
