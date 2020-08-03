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
#include <scene_module/crosswalk/util.h>
#include <utilization/util.h>

#include <cmath>
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

#include <autoware_perception_msgs/DynamicObjectArray.h>

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

bool getBackwordPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

bool insertTargetVelocityPoint(
  const autoware_planning_msgs::PathWithLaneId & input, const Polygon & polygon,
  const double & margin, const double & velocity, const PlannerData & planner_data,
  autoware_planning_msgs::PathWithLaneId & output, DebugData & debug_data,
  boost::optional<int> & first_stop_path_point_index)
{
  output = input;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    const auto p0 = output.points.at(i).point.pose.position;
    const auto p1 = output.points.at(i + 1).point.pose.position;
    const Line line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point> collision_points;
    bg::intersection(polygon, line, collision_points);

    if (collision_points.empty()) continue;
    // -- debug code --
    for (const auto & cp : collision_points) {
      Eigen::Vector3d point3d(cp.x(), cp.y(), planner_data.current_pose.pose.position.z);
      debug_data.collision_points.push_back(point3d);
    }
    std::vector<Eigen::Vector3d> line3d;
    line3d.emplace_back(p0.x, p0.y, p0.z);
    line3d.emplace_back(p1.x, p1.y, p1.z);
    debug_data.collision_lines.push_back(line3d);
    // ----------------

    // check nearest collision point
    Point nearest_collision_point;
    double min_dist;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist = bg::distance(Point(p0.x, p0.y), collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
        debug_data.nearest_collision_point =
          planning_utils::toRosPoint(collision_points.at(j), p0.z);
      }
    }

    // search target point index
    size_t insert_target_point_idx = 0;
    const double base_link2front = planner_data.base_link2front;
    double length_sum = 0;

    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    point1 << nearest_collision_point.x(), nearest_collision_point.y();
    point2 << p0.x, p0.y;
    length_sum += (point2 - point1).norm();
    for (size_t j = i; 0 < j; --j) {
      if (target_length < length_sum) {
        insert_target_point_idx = j + 1;
        break;
      }
      const auto pj1 = output.points.at(j).point.pose.position;
      const auto pj2 = output.points.at(j - 1).point.pose.position;
      point1 << pj1.x, pj1.y;
      point2 << pj2.x, pj2.y;
      length_sum += (point2 - point1).norm();
    }

    // create target point
    Eigen::Vector2d target_point;
    autoware_planning_msgs::PathPointWithLaneId target_point_with_lane_id;
    getBackwordPointFromBasePoint(point2, point1, point2, length_sum - target_length, target_point);
    const int target_velocity_point_idx =
      std::max(static_cast<int>(insert_target_point_idx) - 1, 0);
    target_point_with_lane_id = output.points.at(target_velocity_point_idx);
    target_point_with_lane_id.point.pose.position.x = target_point.x();
    target_point_with_lane_id.point.pose.position.y = target_point.y();
    if (insert_target_point_idx > 0) {
      //calculate z-position of the target point (Internal division point of point1/point2)
      //if insert_target_point_idx is zero, use z-position of target_velocity_point_idx
      const double internal_div_ratio =
        (point1 - target_point).norm() /
        ((point1 - target_point).norm() + (point2 - target_point).norm());
      target_point_with_lane_id.point.pose.position.z =
        output.points.at(insert_target_point_idx).point.pose.position.z * (1 - internal_div_ratio) +
        output.points.at(insert_target_point_idx - 1).point.pose.position.z * internal_div_ratio;
    }

    if ((point1 - point2).norm() > 1.0E-3) {
      const double yaw = std::atan2(point1.y() - point2.y(), point1.x() - point2.x());
      target_point_with_lane_id.point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
    }
    target_point_with_lane_id.point.twist.linear.x = velocity;
    if (velocity == 0.0 && target_velocity_point_idx < first_stop_path_point_index) {
      first_stop_path_point_index = target_velocity_point_idx;
      // -- debug code --
      debug_data.first_stop_pose = target_point_with_lane_id.point.pose;
      // ----------------
    }
    // -- debug code --
    if (velocity == 0.0)
      debug_data.stop_poses.push_back(target_point_with_lane_id.point.pose);
    else
      debug_data.slow_poses.push_back(target_point_with_lane_id.point.pose);
    // ----------------

    // insert target point
    output.points.insert(
      output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

    // insert 0 velocity after target point
    for (size_t j = insert_target_point_idx; j < output.points.size(); ++j)
      output.points.at(j).point.twist.linear.x =
        std::min(velocity, output.points.at(j).point.twist.linear.x);
    return true;
  }
  return false;
}
