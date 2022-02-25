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

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/distance/interface.hpp>
#include <boost/geometry/core/cs.hpp>

#include <algorithm>

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
      // TODO(Maxime CLEMENT): better collision detection
      if (boost::geometry::distance(point, polygon) < 2.0) {
        return true;
      }
    }
  }
  return false;
}

NumberOfViolations checkHardConstraints(Path & path, const Constraints & constraints)
{
  NumberOfViolations number_of_violations;
  if (collideWithPolygons(path, constraints.obstacle_polygons)) {
    ++number_of_violations.collision;
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
