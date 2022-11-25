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

#include "drivable_area_expander/grid_utils.hpp"

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

template <class T>
multipolygon_t createFootprintPolygons(
  const T & path, const double front, const double rear, const double left, const double right)
{
  multipolygon_t footprint;
  polygon_t base_polygon;
  base_polygon.outer() = {
    point_t{front, left}, point_t{front, right}, point_t{rear, right}, point_t{rear, left}};
  base_polygon.outer().push_back(base_polygon.outer().front());
  for (const auto & p : path) {
    const auto & pose = getPose(p);
    const auto angle = tf2::getYaw(pose.orientation);
    const auto polygon =
      translatePolygon(rotatePolygon(base_polygon, angle), pose.position.x, pose.position.y);
    footprint.push_back(polygon);
  }
  return footprint;
}

multipolygon_t createPathFootprint(const Path & path, const ExpansionParameters & params)
{
  const auto left = params.vehicle_left_offset_ + params.extra_footprint_offset;
  const auto right = params.vehicle_right_offset_ - params.extra_footprint_offset;
  const auto rear = params.vehicle_rear_offset_ - params.extra_footprint_offset;
  const auto front = params.vehicle_front_offset_ + params.extra_footprint_offset;
  return createFootprintPolygons(path.points, front, rear, left, right);
}

multipolygon_t createObjectFootprints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  double min_velocity)
{
  multipolygon_t footprints;
  for (const auto & object : objects.objects) {
    if (std::abs(object.kinematics.initial_twist_with_covariance.twist.linear.x >= min_velocity)) {
      const auto front = object.shape.dimensions.x / 2 + buffer;
      const auto rear = -front;
      const auto left = object.shape.dimensions.y / 2 + buffer;
      const auto right = -left;
      for (const auto & path : object.kinematics.predicted_paths) {
        const auto footprint = createFootprintPolygons(path.path, front, rear, left, right);
        footprints.insert(footprints.end(), footprint.begin(), footprint.end());
      }
    }
  }
  return footprints;
}
}  // namespace drivable_area_expander
