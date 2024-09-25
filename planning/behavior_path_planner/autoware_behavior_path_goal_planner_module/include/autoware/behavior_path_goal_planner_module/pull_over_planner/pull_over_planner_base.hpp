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

#pragma once

#include "autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <utility>
#include <vector>

using autoware::universe_utils::LinearRing2d;
using geometry_msgs::msg::Pose;
using tier4_planning_msgs::msg::PathWithLaneId;

namespace autoware::behavior_path_planner
{
enum class PullOverPlannerType {
  SHIFT,
  ARC_FORWARD,
  ARC_BACKWARD,
  FREESPACE,
};

struct PullOverPath
{
public:
  static std::optional<PullOverPath> create(
    const PullOverPlannerType & type, const size_t goal_id, const size_t id,
    const std::vector<PathWithLaneId> & partial_paths, const Pose & start_pose,
    const Pose & end_pose)
  {
    if (partial_paths.empty()) {
      return std::nullopt;
    }
    PathWithLaneId path{};
    for (size_t i = 0; i < partial_paths.size(); ++i) {
      if (i == 0) {
        path.points.insert(
          path.points.end(), partial_paths.at(i).points.begin(), partial_paths.at(i).points.end());
      } else {
        // skip overlapping point
        path.points.insert(
          path.points.end(), next(partial_paths.at(i).points.begin()),
          partial_paths.at(i).points.end());
      }
    }
    PathWithLaneId full_path{};
    full_path.points = autoware::motion_utils::removeOverlapPoints(path.points);
    if (full_path.points.size() < 3) {
      return std::nullopt;
    }

    const size_t start_idx =
      autoware::motion_utils::findNearestIndex(full_path.points, start_pose.position);

    PathWithLaneId parking_path{};
    std::copy(
      full_path.points.begin() + start_idx, full_path.points.end(),
      std::back_inserter(parking_path.points));

    if (parking_path.points.size() < 3) {
      return std::nullopt;
    }

    const auto calculateCurvaturesAndMax =
      [](const auto & path) -> std::pair<std::vector<double>, double> {
      std::vector<double> curvatures = autoware::motion_utils::calcCurvature(path.points);
      double max_curvature = 0.0;
      if (!curvatures.empty()) {
        max_curvature = std::abs(*std::max_element(
          curvatures.begin(), curvatures.end(),
          [](const double & a, const double & b) { return std::abs(a) < std::abs(b); }));
      }
      return std::make_pair(curvatures, max_curvature);
    };

    std::vector<double> full_path_curvatures{};
    std::vector<double> parking_path_curvatures{};
    double full_path_max_curvature{};
    double parking_path_max_curvature{};
    std::tie(full_path_curvatures, full_path_max_curvature) = calculateCurvaturesAndMax(full_path);
    std::tie(parking_path_curvatures, parking_path_max_curvature) =
      calculateCurvaturesAndMax(full_path);

    return PullOverPath(
      type, goal_id, id, start_pose, end_pose, partial_paths, std::move(full_path),
      std::move(parking_path), std::move(full_path_curvatures), std::move(parking_path_curvatures),
      full_path_max_curvature, full_path_max_curvature, parking_path_max_curvature);
  }

  PullOverPath(const PullOverPath &) = default;

  const PullOverPlannerType type;
  const size_t goal_id;
  const size_t id;
  const Pose start_pose;
  const Pose end_pose;

  // todo: multithreadでこれを参照するとアウト
  const std::vector<PathWithLaneId> partial_paths;
  const PathWithLaneId full_path;
  const PathWithLaneId parking_path;

  const std::vector<double> full_path_curvatures;
  const std::vector<double> parking_path_curvatures;
  const double full_path_max_curvature;
  const double parking_path_max_curvature;

private:
  PullOverPath(
    const PullOverPlannerType & type, const size_t goal_id, const size_t id,
    const Pose & start_pose, const Pose & end_pose, std::vector<PathWithLaneId> && partial_paths,
    PathWithLaneId && full_path, PathWithLaneId && parking_path,
    std::vector<double> && full_path_curvatures, std::vector<double> && parking_path_curvatures,
    const double full_path_max_curvature, const double parking_path_max_curvature)
  : type(type),
    goal_id(goal_id),
    id(id),
    start_pose(start_pose),
    end_pose(end_pose),
    partial_paths(partial_paths),
    full_path(full_path),
    parking_path(parking_path),
    full_path_curvatures(full_path_curvatures),
    parking_path_curvatures(parking_path_curvatures),
    full_path_max_curvature(full_path_max_curvature),
    parking_path_max_curvature(parking_path_max_curvature)
  {
  }

  size_t path_idx{0};
  // accelerate with constant acceleration to the target velocity
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel{};
  std::vector<Pose> debug_poses{};

  const PathWithLaneId & getCurrentPath() const
  {
    if (partial_paths.size() <= path_idx) {
      return partial_paths.back();
    }
    return partial_paths.at(path_idx);
  }

  // この中でcurrent_pathを更新する
  bool incrementPathIndex()
  {
    if (partial_paths.size() - 1 <= path_idx) {
      return false;
    }
    path_idx += 1;
    return true;
  }
};

class PullOverPlannerBase
{
public:
  PullOverPlannerBase(rclcpp::Node & node, const GoalPlannerParameters & parameters)
  : vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo()},
    vehicle_footprint_{vehicle_info_.createFootprint()},
    parameters_{parameters}
  {
  }
  virtual ~PullOverPlannerBase() = default;

  virtual PullOverPlannerType getPlannerType() const = 0;
  virtual std::optional<PullOverPath> plan(
    const std::shared_ptr<const PlannerData> planner_data,
    const BehaviorModuleOutput & previous_module_output, const Pose & goal_pose) = 0;

protected:
  const autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  const LinearRing2d vehicle_footprint_;
  const GoalPlannerParameters parameters_;
};
}  // namespace autoware::behavior_path_planner
