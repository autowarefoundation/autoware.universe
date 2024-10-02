// Copyright 2024 Tier IV, Inc.
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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <vector>

namespace autoware::behavior_velocity_planner::no_stopping_area
{

bool is_target_stuck_vehicle_type(const autoware_perception_msgs::msg::PredictedObject & object)
{
  return object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::CAR ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::BUS ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::TRUCK ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::TRAILER ||
         object.classification.front().label ==
           autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
}

void insert_stop_point(
  tier4_planning_msgs::msg::PathWithLaneId & path, const PathIndexWithPose & stop_point)
{
  auto insert_idx = static_cast<size_t>(stop_point.first + 1);
  const auto stop_pose = stop_point.second;

  // To PathPointWithLaneId
  tier4_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = path.points.at(insert_idx);
  stop_point_with_lane_id.point.pose = stop_pose;
  stop_point_with_lane_id.point.longitudinal_velocity_mps = 0.0;

  // Insert stop point or replace with zero velocity
  planning_utils::insertVelocity(path, stop_point_with_lane_id, 0.0, insert_idx);
}

std::optional<LineString2d> generate_stop_line(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstPolygons3d & no_stopping_areas, const double ego_width,
  const double stop_line_margin)
{
  LineString2d stop_line;
  for (const auto & no_stopping_area : no_stopping_areas) {
    const auto & area_poly = lanelet::utils::to2D(no_stopping_area).basicPolygon();
    lanelet::BasicLineString2d path_line;
    for (size_t i = 0; i < path.points.size() - 1; ++i) {
      const auto p0 = path.points.at(i).point.pose.position;
      const auto p1 = path.points.at(i + 1).point.pose.position;
      const LineString2d line{{p0.x, p0.y}, {p1.x, p1.y}};
      std::vector<Point2d> collision_points;
      boost::geometry::intersection(area_poly, line, collision_points);
      if (!collision_points.empty()) {
        const double yaw = autoware::universe_utils::calcAzimuthAngle(p0, p1);
        const double w = ego_width;
        const double l = stop_line_margin;
        stop_line.emplace_back(
          -l * std::cos(yaw) + collision_points.front().x() + w * std::cos(yaw + M_PI_2),
          collision_points.front().y() + w * std::sin(yaw + M_PI_2));
        stop_line.emplace_back(
          -l * std::cos(yaw) + collision_points.front().x() + w * std::cos(yaw - M_PI_2),
          collision_points.front().y() + w * std::sin(yaw - M_PI_2));
        return stop_line;
      }
    }
  }
  return {};
}
}  // namespace autoware::behavior_velocity_planner::no_stopping_area
