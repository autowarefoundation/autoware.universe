// Copyright 2022 Tier IV, Inc.
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

#include "sampler_common/constraints/hard_constraint.hpp"

#include "sampler_common/constraints/path_footprint.hpp"
#include "sampler_common/structures.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/core/cs.hpp>

#include <algorithm>
#include <csignal>
#include <iostream>

namespace sampler_common::constraints
{
bool satisfyMinMax(const std::vector<double> & values, const double min, const double max)
{
  for (const auto value : values) {
    if (value < min || value > max) return false;
  }
  return true;
}

bool collideWithPolygons(const Path & path, const std::vector<Polygon> & polygons)
{
  for (const auto & polygon : polygons) {
    for (const auto & point : path.points) {
      if (boost::geometry::distance(point, polygon) < 1.0) {
        return true;
      }
    }
  }
  return false;
}

bool collideWithPolygons(const Polygon & footprint, const std::vector<Polygon> & polygons)
{
  for (const auto & polygon : polygons) {
    if (boost::geometry::intersects(footprint, polygon)) {
      return true;
    }
  }
  return false;
}

bool withinPolygons(const Path & path, const std::vector<Polygon> & polygons)
{
  for (const auto & point : path.points) {
    bool within_at_least_one_poly = false;
    for (const auto & polygon : polygons) {
      if (boost::geometry::within(point, polygon.outer())) {
        within_at_least_one_poly = true;
        break;
      }
    }
    if (!within_at_least_one_poly) return false;
  }
  return true;
}

bool withinPolygons(const Polygon & footprint, const Polygon & polygons)
{
  return boost::geometry::within(footprint, polygons);
}

NumberOfViolations checkHardConstraints(Path & path, const Constraints & constraints)
{
  // TODO(Maxime CLEMENT): get from vehicle parameters
  const Polygon footprint = buildFootprintPolygon(path, constraints);
  NumberOfViolations number_of_violations;
  if (collideWithPolygons(footprint, constraints.obstacle_polygons)) {
    ++number_of_violations.collision;
    path.valid = false;
  }
  if (!withinPolygons(footprint, constraints.drivable_polygon)) {
    ++number_of_violations.outside;
    path.valid = false;
  }
  if (!satisfyMinMax(
        path.curvatures, constraints.hard.min_curvature, constraints.hard.max_curvature)) {
    ++number_of_violations.curvature;
    path.valid = false;
  }
  return number_of_violations;
}

}  // namespace sampler_common::constraints
