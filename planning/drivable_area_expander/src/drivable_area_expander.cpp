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

#include "drivable_area_expander/drivable_area_expander.hpp"

#include "drivable_area_expander/parameters.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

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

polygon_t createPathFootprint(const Path & path, const ExpansionParameters & params)
{
  multipolygon_t unions;
  multipolygon_t result;
  const auto left = params.vehicle_left_offset_ + params.extra_footprint_offset;
  const auto right = params.vehicle_right_offset_ - params.extra_footprint_offset;
  const auto rear = params.vehicle_rear_offset_ - params.extra_footprint_offset;
  const auto front = params.vehicle_front_offset_ + params.extra_footprint_offset;
  polygon_t base_polygon;
  base_polygon.outer() = {
    point_t{front, left}, point_t{front, right}, point_t{rear, right}, point_t{rear, left}};
  base_polygon.outer().push_back(base_polygon.outer().front());
  for (const auto & p : path.points) {
    const auto angle = tf2::getYaw(p.pose.orientation);
    const auto polygon =
      translatePolygon(rotatePolygon(base_polygon, angle), p.pose.position.x, p.pose.position.y);
    boost::geometry::union_(polygon, unions, result);
    unions = result;
    boost::geometry::clear(result);
  }
  if (unions.empty()) return {};
  return unions.back();
}

}  // namespace drivable_area_expander
