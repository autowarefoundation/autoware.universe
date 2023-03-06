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

#include "obstacle_cruise_planner/polygon_utils.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

namespace
{
void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point(geom_point.x, geom_point.y);
  bg::append(polygon.outer(), point);
}

geometry_msgs::msg::Point calcOffsetPosition(
  const geometry_msgs::msg::Pose & pose, const double offset_x, const double offset_y)
{
  return tier4_autoware_utils::calcOffsetPose(pose, offset_x, offset_y, 0.0).position;
}

Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  Polygon2d polygon;

  const double base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  // base step
  appendPointToPolygon(polygon, calcOffsetPosition(base_step_pose, base_to_front, width));
  appendPointToPolygon(polygon, calcOffsetPosition(base_step_pose, base_to_front, -width));
  appendPointToPolygon(polygon, calcOffsetPosition(base_step_pose, -base_to_rear, -width));
  appendPointToPolygon(polygon, calcOffsetPosition(base_step_pose, -base_to_rear, width));

  // next step
  appendPointToPolygon(polygon, calcOffsetPosition(next_step_pose, base_to_front, width));
  appendPointToPolygon(polygon, calcOffsetPosition(next_step_pose, base_to_front, -width));
  appendPointToPolygon(polygon, calcOffsetPosition(next_step_pose, -base_to_rear, -width));
  appendPointToPolygon(polygon, calcOffsetPosition(next_step_pose, -base_to_rear, width));

  bg::correct(polygon);

  Polygon2d hull_polygon;
  bg::convex_hull(polygon, hull_polygon);

  return hull_polygon;
}

PointWithStamp calcNearestCollisionPoint(
  const size_t & first_within_idx, const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points, const double vehicle_lon_offset,
  const bool is_driving_forward)
{
  std::vector<geometry_msgs::msg::Point> segment_points(2);
  if (first_within_idx == 0) {
    const auto & traj_front_pose = decimated_traj_points.at(0).pose;
    const auto front_pos =
      tier4_autoware_utils::calcOffsetPose(traj_front_pose, vehicle_lon_offset, 0.0, 0.0).position;
    if (is_driving_forward) {
      segment_points.at(0) = traj_front_pose.position;
      segment_points.at(1) = front_pos;
    } else {
      segment_points.at(0) = front_pos;
      segment_points.at(1) = traj_front_pose.position;
    }
  } else {
    const size_t seg_idx = first_within_idx - 1;
    segment_points.at(0) = decimated_traj_points.at(seg_idx).pose.position;
    segment_points.at(1) = decimated_traj_points.at(seg_idx + 1).pose.position;
  }

  size_t min_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t cp_idx = 0; cp_idx < collision_points.size(); ++cp_idx) {
    const auto & collision_point = collision_points.at(cp_idx);
    const double dist =
      motion_utils::calcLongitudinalOffsetToSegment(segment_points, 0, collision_point.point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = cp_idx;
    }
  }

  return collision_points.at(min_idx);
}
}  // namespace

namespace polygon_utils
{
std::optional<std::pair<size_t, std::vector<PointWithStamp>>> getCollisionIndex(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Pose & pose, const rclcpp::Time & stamp, const Shape & shape,
  const double max_dist)
{
  const auto obstacle_polygon = tier4_autoware_utils::toPolygon2d(pose, shape);
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double approximated_dist =
      tier4_autoware_utils::calcDistance2d(traj_points.at(i).pose, pose);
    if (approximated_dist > max_dist) {
      continue;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obstacle_polygon, collision_polygons);

    std::vector<PointWithStamp> collision_geom_points;
    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;

        for (const auto & collision_point : collision_polygon.outer()) {
          PointWithStamp collision_geom_point;
          collision_geom_point.stamp = stamp;  // TODO(murooka) stamp with same stamp?
          collision_geom_point.point.x = collision_point.x();
          collision_geom_point.point.y = collision_point.y();
          collision_geom_points.push_back(collision_geom_point);
        }
      }
    }

    if (has_collision) {
      const auto collision_info =
        std::pair<size_t, std::vector<PointWithStamp>>{i, collision_geom_points};
      return collision_info;
    }
  }

  return std::nullopt;
}

std::optional<geometry_msgs::msg::Point> getCollisionPoint(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const Obstacle & obstacle, const double vehicle_lon_offset, const bool is_driving_forward,
  const double max_dist)
{
  const auto collision_info = getCollisionIndex(
    traj_points, traj_polygons, obstacle.pose, obstacle.stamp, obstacle.shape, max_dist);
  if (collision_info) {
    const auto nearest_collision_point = calcNearestCollisionPoint(
      collision_info->first, collision_info->second, traj_points, vehicle_lon_offset,
      is_driving_forward);
    return nearest_collision_point.point;
  }

  return std::nullopt;
}

std::vector<PointWithStamp> getCollisionPoints(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const double vehicle_lon_offset, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_dist,
  const double max_prediction_time_for_collision_check)
{
  std::vector<PointWithStamp> collision_points;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    if (
      max_prediction_time_for_collision_check <
      rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(i)) {
      break;
    }

    const auto object_time =
      rclcpp::Time(obstacle_stamp) + rclcpp::Duration(predicted_path.time_step) * i;
    // Ignore past position
    if ((object_time - current_time).seconds() < 0.0) {
      continue;
    }

    const auto collision_info = getCollisionIndex(
      traj_points, traj_polygons, predicted_path.path.at(i), object_time, shape, max_dist);
    if (collision_info) {
      const auto nearest_collision_point = calcNearestCollisionPoint(
        collision_info->first, collision_info->second, traj_points, vehicle_lon_offset,
        is_driving_forward);
      collision_points.push_back(nearest_collision_point);
      collision_index.push_back(collision_info->first);
    }
  }

  return collision_points;
}

std::vector<PointWithStamp> willCollideWithSurroundObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const double max_dist,
  const double ego_obstacle_overlap_time_threshold,
  const double max_prediction_time_for_collision_check, std::vector<size_t> & collision_index,
  const double vehicle_lon_offset, const bool is_driving_forward)
{
  const auto collision_points = getCollisionPoints(
    traj_points, traj_polygons, obstacle_stamp, predicted_path, shape, current_time,
    vehicle_lon_offset, is_driving_forward, collision_index, max_dist,
    max_prediction_time_for_collision_check);

  if (collision_points.empty()) {
    return {};
  }

  const double overlap_time =
    (rclcpp::Time(collision_points.back().stamp) - rclcpp::Time(collision_points.front().stamp))
      .seconds();
  if (overlap_time < ego_obstacle_overlap_time_threshold) {
    return {};
  }

  return collision_points;
}

std::vector<Polygon2d> createOneStepPolygons(
  const std::vector<TrajectoryPoint> & traj_points,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  std::vector<Polygon2d> polygons;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto polygon = [&]() {
      if (i == 0) {
        return createOneStepPolygon(traj_points.at(i).pose, traj_points.at(i).pose, vehicle_info);
      }
      return createOneStepPolygon(traj_points.at(i - 1).pose, traj_points.at(i).pose, vehicle_info);
    }();

    polygons.push_back(polygon);
  }
  return polygons;
}

}  // namespace polygon_utils
