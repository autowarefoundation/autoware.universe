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

#include <sampler_common/constraints/footprint.hpp>
#include <sampler_common/structures.hpp>

#include <boost/geometry/io/wkt/write.hpp>

#include <gtest/gtest.h>

using sampler_common::Constraints;
using sampler_common::Path;
using sampler_common::Point;
using sampler_common::Polygon;
using sampler_common::constraints::buildFootprintPolygon;

const auto point_in_polygon = [](const Point & p, const Polygon & poly) {
  constexpr auto EPS = 1e-6;
  for (const auto & poly_p : poly.outer()) {
    if (
      poly_p.x() < p.x() + EPS && poly_p.x() > p.x() - EPS && poly_p.y() < p.y() + EPS &&
      poly_p.y() > p.y() - EPS)
      return true;
  }
  return false;
};

TEST(Footprint, pointFootprint)
{
  Constraints c;
  // square footprint
  c.vehicle_offsets.left_front = {1.0, 1.0};
  c.vehicle_offsets.right_front = {1.0, -1.0};
  c.vehicle_offsets.left_rear = {-1.0, 1.0};
  c.vehicle_offsets.right_rear = {-1.0, -1.0};
  {
    const Point p = {0.0, 0.0};
    for (auto orientation : {0.0, M_PI_2, M_PI, 3 * M_PI_2}) {
      const auto polygon = buildFootprintPolygon(p, orientation, c);
      EXPECT_EQ(polygon.outer().size(), 5ul);
      EXPECT_TRUE(point_in_polygon({1.0, 1.0}, polygon));
      EXPECT_TRUE(point_in_polygon({1.0, -1.0}, polygon));
      EXPECT_TRUE(point_in_polygon({-1.0, 1.0}, polygon));
      EXPECT_TRUE(point_in_polygon({-1.0, -1.0}, polygon));
    }
    for (auto orientation : {M_PI_4, 3 * M_PI_4, 5 * M_PI_4, 7 * M_PI_4}) {
      const auto polygon = buildFootprintPolygon(p, orientation, c);
      std::cout << boost::geometry::wkt(polygon.outer()) << std::endl;
      EXPECT_EQ(polygon.outer().size(), 5ul);
      EXPECT_TRUE(point_in_polygon({std::sqrt(2), std::sqrt(2)}, polygon));
      EXPECT_TRUE(point_in_polygon({std::sqrt(2), -std::sqrt(2)}, polygon));
      EXPECT_TRUE(point_in_polygon({-std::sqrt(2), std::sqrt(2)}, polygon));
      EXPECT_TRUE(point_in_polygon({-std::sqrt(2), -std::sqrt(2)}, polygon));
    }
  }
  {
    const Point p = {2.0, 5.0};
    for (auto orientation : {0.0, M_PI_2, M_PI, 3 * M_PI_2}) {
      const auto polygon = buildFootprintPolygon(p, orientation, c);
      EXPECT_EQ(polygon.outer().size(), 5ul);
      EXPECT_TRUE(point_in_polygon({3.0, 6.0}, polygon));
      EXPECT_TRUE(point_in_polygon({3.0, 4.0}, polygon));
      EXPECT_TRUE(point_in_polygon({1.0, 6.0}, polygon));
      EXPECT_TRUE(point_in_polygon({1.0, 4.0}, polygon));
    }
  }
}

TEST(Footprint, pathFootprint) {}
