// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_CROSSWALK_MODULE__UTIL_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_CROSSWALK_MODULE__UTIL_HPP_

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/stop_factor.hpp>

namespace autoware::behavior_velocity_planner
{

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::StopFactor;

enum class CollisionState { YIELD, EGO_PASS_FIRST, EGO_PASS_LATER, IGNORE };

struct CollisionPoint
{
  geometry_msgs::msg::Point collision_point{};
  std::optional<double> crosswalk_passage_direction{
    std::nullopt};  // denote obj is passing the crosswalk along the vehicle lane
  double time_to_collision{};
  double time_to_vehicle{};
};

struct DebugData
{
  DebugData() = default;
  explicit DebugData(const std::shared_ptr<const PlannerData> planner_data)
  : base_link2front(planner_data->vehicle_info_.max_longitudinal_offset_m)
  {
  }

  bool ignore_crosswalk{false};
  double base_link2front;
  double stop_judge_range{};
  std::string virtual_wall_suffix{};

  geometry_msgs::msg::Pose first_stop_pose;
  geometry_msgs::msg::Point nearest_collision_point;

  std::optional<geometry_msgs::msg::Point> range_near_point{std::nullopt};
  std::optional<geometry_msgs::msg::Point> range_far_point{std::nullopt};

  std::vector<std::tuple<std::string, CollisionPoint, CollisionState>> collision_points;

  std::vector<geometry_msgs::msg::Pose> stop_poses;
  std::vector<geometry_msgs::msg::Pose> slow_poses;
  std::vector<geometry_msgs::msg::Pose> pass_poses;
  std::vector<geometry_msgs::msg::Point> stop_factor_points;
  std::vector<geometry_msgs::msg::Point> crosswalk_polygon;
  std::vector<std::vector<geometry_msgs::msg::Point>> ego_polygons;
  std::vector<std::vector<geometry_msgs::msg::Point>> obj_polygons;

  // occlusion data
  std::vector<lanelet::BasicPolygon2d> occlusion_detection_areas;
  geometry_msgs::msg::Point crosswalk_origin;
};

std::vector<std::pair<int64_t, lanelet::ConstLanelet>> getCrosswalksOnPath(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs);

std::set<int64_t> getCrosswalkIdSetOnPath(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs);

std::optional<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>
getPathEndPointsOnCrosswalk(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos);

std::vector<geometry_msgs::msg::Point> getLinestringIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicLineString2d & linestring,
  const geometry_msgs::msg::Point & ego_pos);

std::optional<lanelet::ConstLineString3d> getStopLineFromMap(
  const lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name);
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_CROSSWALK_MODULE__UTIL_HPP_
