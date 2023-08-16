// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/drivable_area_expansion/one_step_polygons.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

namespace polygon_utils
{

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
  const vehicle_info_util::VehicleInfo & vehicle_info, const double lat_margin)
{
  Polygon2d polygon;

  const double base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0 + lat_margin;
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
}  // namespace

std::vector<Polygon2d> createOneStepPolygons(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double lat_margin)
{
  std::vector<Polygon2d> polygons;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto polygon = [&]() {
      if (i == 0) {
        return createOneStepPolygon(
          traj_points.at(i).pose, traj_points.at(i).pose, vehicle_info, lat_margin);
      }
      return createOneStepPolygon(
        traj_points.at(i - 1).pose, traj_points.at(i).pose, vehicle_info, lat_margin);
    }();

    polygons.push_back(polygon);
  }
  return polygons;
}
}  // namespace polygon_utils
