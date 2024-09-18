// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <optional>
#include <vector>

namespace autoware::universe_utils
{

using Polygon2d = autoware::universe_utils::Polygon2d;
using Point2d = autoware::universe_utils::Point2d;
using LinearRing2d = autoware::universe_utils::LinearRing2d;

struct LinkedPoint
{
  LinkedPoint(std::size_t index, const Point2d & point)
  : i(index), pt(point), steiner(false), prev_index(std::nullopt), next_index(std::nullopt)
  {
  }

  std::size_t i;
  Point2d pt;
  bool steiner;
  std::optional<std::size_t> prev_index;
  std::optional<std::size_t> next_index;
  [[nodiscard]] double x() const { return pt.x(); }
  [[nodiscard]] double y() const { return pt.y(); }
};
void ear_clipping_linked(
  std::optional<std::size_t> ear_index, std::vector<std::size_t> & indices,
  std::vector<LinkedPoint> & points, int pass = 0);
void split_ear_clipping(
  std::vector<LinkedPoint> & points, std::size_t start_index, std::vector<std::size_t> & indices);

std::vector<Polygon2d> triangulate(const Polygon2d & polygon);
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
