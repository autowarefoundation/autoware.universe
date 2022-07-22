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

#ifndef SCENE_MODULE__SPEED_BUMP__UTIL_HPP_
#define SCENE_MODULE__SPEED_BUMP__UTIL_HPP_

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "behavior_velocity_planner/planner_data.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

namespace behavior_velocity_planner
{
namespace speed_bump_util
{
struct DebugData
{
  double base_link2front;
  std::vector<Eigen::Vector3d> collision_points;
  std::vector<geometry_msgs::msg::Pose> slow_poses;
  std::vector<geometry_msgs::msg::Pose> acc_poses;
  std::vector<std::vector<Eigen::Vector3d>> collision_lines;
  std::vector<std::vector<Eigen::Vector3d>> slow_polygons;
  geometry_msgs::msg::Point nearest_collision_point;
};

bool insertTargetVelocityPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> & polygon,
  const double & slow_margin, const double & acc_margin, const double & bump_height,
  const PlannerData & planner_data, autoware_auto_planning_msgs::msg::PathWithLaneId & output,
  DebugData & debug_data);

double velocityByBumpHeight(double bump_height);

void setVelocityFromIndex(
  const size_t begin_idx, const double vel,
  autoware_auto_planning_msgs::msg::PathWithLaneId & input);

}  // namespace speed_bump_util
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__SPEED_BUMP__UTIL_HPP_
