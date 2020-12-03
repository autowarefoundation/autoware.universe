/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#pragma once

#include <string>
#include <vector>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <behavior_velocity_planner/planner_data.hpp>
#include <autoware_planning_msgs/msg/path_with_lane_id.hpp>

struct DebugData
{
  double base_link2front;
  std::vector<Eigen::Vector3d> collision_points;
  geometry_msgs::msg::Pose first_stop_pose;
  std::vector<geometry_msgs::msg::Pose> stop_poses;
  std::vector<geometry_msgs::msg::Pose> slow_poses;
  std::vector<std::vector<Eigen::Vector3d>> collision_lines;
  std::vector<std::vector<Eigen::Vector3d>> crosswalk_polygons;
  std::vector<std::vector<Eigen::Vector3d>> stop_polygons;
  std::vector<geometry_msgs::msg::Point> stop_factor_points;
  std::vector<std::vector<Eigen::Vector3d>> slow_polygons;
  geometry_msgs::msg::Point nearest_collision_point;
};

bool insertTargetVelocityPoint(
  const autoware_planning_msgs::msg::PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
    polygon,
  const double & margin, const double & velocity, const PlannerData & planner_data,
  autoware_planning_msgs::msg::PathWithLaneId & output, DebugData & debug_data,
  boost::optional<int> & first_stop_path_point_index);
