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

#include "drivable_area_expander/obstacles.hpp"

#include "drivable_area_expander/parameters.hpp"

#include <boost/assign.hpp>
#include <boost/geometry.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/utils.h>

namespace drivable_area_expander
{
polygon_t rotatePolygon(const polygon_t & polygon, const double angle)
{
  polygon_t rotated_polygon;
  const boost::geometry::strategy::transform::rotate_transformer<
    boost::geometry::radian, double, 2, 2>
    rotation(-angle);
  boost::geometry::transform(polygon, rotated_polygon, rotation);
  return rotated_polygon;
}

polygon_t translatePolygon(const polygon_t & polygon, const double x, const double y)
{
  polygon_t translated_polygon;
  const boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translation(x, y);
  boost::geometry::transform(polygon, translated_polygon, translation);
  return translated_polygon;
}

Footprint createFootprint(const geometry_msgs::msg::Pose & pose, const polygon_t base_footprint)
{
  const auto angle = tf2::getYaw(pose.orientation);
  const auto polygon =
    translatePolygon(rotatePolygon(base_footprint, angle), pose.position.x, pose.position.y);
  return Footprint(polygon, point_t{pose.position.x, pose.position.y});
}

std::vector<Footprint> createObjectFootprints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const ExpansionParameters & params)
{
  std::vector<Footprint> footprints;
  for (const auto & object : objects.objects) {
    const auto front = object.shape.dimensions.x / 2 + params.dynamic_objects_extra_front_offset;
    const auto rear = -object.shape.dimensions.x / 2 - params.dynamic_objects_extra_rear_offset;
    const auto left = object.shape.dimensions.y / 2 + params.dynamic_objects_extra_left_offset;
    const auto right = -object.shape.dimensions.y / 2 - params.dynamic_objects_extra_right_offset;
    polygon_t base_footprint;
    base_footprint.outer() = {
      point_t{front, left}, point_t{front, right}, point_t{rear, right}, point_t{rear, left},
      point_t{front, left}};
    for (const auto & path : object.kinematics.predicted_paths)
      for (const auto & pose : path.path)
        footprints.push_back(createFootprint(pose, base_footprint));
  }
  return footprints;
}
}  // namespace drivable_area_expander
