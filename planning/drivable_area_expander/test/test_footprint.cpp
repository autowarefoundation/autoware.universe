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
#include "drivable_area_expander/types.hpp"

#include <boost/geometry/io/wkt/wkt.hpp>

#include <gtest/gtest.h>

TEST(TestFootprint, rotatePolygon)
{
  using drivable_area_expander::polygon_t;
  using drivable_area_expander::rotatePolygon;
  polygon_t poly;
  poly.outer() = {{1, 1}, {1, -1}, {-2, -1}, {-2, 1}, {1, 1}};
  {
    const auto rot = rotatePolygon(poly, M_PI_2);
    ASSERT_EQ(rot.outer().size(), poly.outer().size());
  }
  {
    const auto rot = rotatePolygon(poly, M_PI);
    ASSERT_EQ(rot.outer().size(), poly.outer().size());
  }
  {
    const auto rot = rotatePolygon(poly, M_PI_2 * 3);
    ASSERT_EQ(rot.outer().size(), poly.outer().size());
  }
  {
    const auto rot = rotatePolygon(poly, 2 * M_PI);
    ASSERT_EQ(rot.outer().size(), poly.outer().size());
  }
}
