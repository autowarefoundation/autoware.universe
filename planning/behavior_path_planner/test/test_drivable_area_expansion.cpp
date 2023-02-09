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

#include "behavior_path_planner/util/drivable_area_expansion/path_projection.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/types.hpp"

#include <gtest/gtest.h>

using drivable_area_expansion::linestring_t;
using drivable_area_expansion::point_t;
using drivable_area_expansion::segment_t;

TEST(DrivableAreaExpansionProjection, PointToSegment)
{
  using drivable_area_expansion::point_to_segment_projection;

  {
    point_t query(1.0, 1.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_EQ(projection.distance, 1.0);
    EXPECT_EQ(projection.point.x(), 1.0);
    EXPECT_EQ(projection.point.y(), 0.0);
  }
  {
    point_t query(-1.0, 1.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_EQ(projection.distance, std::sqrt(2));
    EXPECT_EQ(projection.point.x(), 0.0);
    EXPECT_EQ(projection.point.y(), 0.0);
  }
  {
    point_t query(11.0, 1.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_EQ(projection.distance, std::sqrt(2));
    EXPECT_EQ(projection.point.x(), 10.0);
    EXPECT_EQ(projection.point.y(), 0.0);
  }
  {
    point_t query(5.0, -5.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_EQ(projection.distance, -5.0);
    EXPECT_EQ(projection.point.x(), 5.0);
    EXPECT_EQ(projection.point.y(), 0.0);
  }
  {
    point_t query(5.0, -5.0);
    segment_t segment(point_t(0.0, 0.0), point_t(0.0, -10.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_EQ(projection.distance, 5.0);
    EXPECT_EQ(projection.point.x(), 0.0);
    EXPECT_EQ(projection.point.y(), -5.0);
  }
  {
    point_t query(5.0, 5.0);
    segment_t segment(point_t(2.5, 7.5), point_t(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_EQ(projection.distance, 0.0);
    EXPECT_EQ(projection.point.x(), 5.0);
    EXPECT_EQ(projection.point.y(), 5.0);
  }
  {
    point_t query(0.0, 0.0);
    segment_t segment(point_t(2.5, 7.5), point_t(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_EQ(projection.distance, -std::sqrt(50));
    EXPECT_EQ(projection.point.x(), 5.0);
    EXPECT_EQ(projection.point.y(), 5.0);
  }
}

TEST(DrivableAreaExpansionProjection, PointToLinestring)
{
  using drivable_area_expansion::point_to_linestring_projection;

  linestring_t ls = {
    point_t(0.0, 0.0), point_t(10.0, 0.0), point_t(10.0, 10.0), point_t(0.0, 10.0),
    point_t(5.0, 5.0)};
  {
    point_t query(0.0, 0.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_EQ(projection.arc_length, 0.0);
    EXPECT_EQ(projection.distance, 0.0);
    EXPECT_EQ(projection.projected_point.x(), 0.0);
    EXPECT_EQ(projection.projected_point.y(), 0.0);
  }
  {
    point_t query(2.0, 1.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_EQ(projection.arc_length, 2.0);
    EXPECT_EQ(projection.distance, 1.0);
    EXPECT_EQ(projection.projected_point.x(), 2.0);
    EXPECT_EQ(projection.projected_point.y(), 0.0);
  }
  {
    point_t query(0.0, 5.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_EQ(projection.arc_length, 30.0 + std::sqrt(2.5 * 2.5 * 2));
    EXPECT_EQ(projection.distance, -std::sqrt(2.5 * 2.5 * 2));
    EXPECT_EQ(projection.projected_point.x(), 2.5);
    EXPECT_EQ(projection.projected_point.y(), 7.5);
  }
}

TEST(DrivableAreaExpansionProjection, LinestringToPoint)
{
  using drivable_area_expansion::linestring_to_point_projection;

  linestring_t ls = {
    point_t(0.0, 0.0), point_t(10.0, 0.0), point_t(10.0, 10.0), point_t(0.0, 10.0),
    point_t(5.0, 5.0)};
  for (auto arc_length = 0.0; arc_length <= 10.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_EQ(projection.first.x(), arc_length);
    EXPECT_EQ(projection.first.y(), 0.0);
    EXPECT_EQ(projection.second.x(), arc_length);
    EXPECT_EQ(projection.second.y(), 0.0);
  }
  for (auto arc_length = 11.0; arc_length <= 20.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_EQ(projection.first.x(), 10.0);
    EXPECT_EQ(projection.first.y(), arc_length - 10.0);
    EXPECT_EQ(projection.second.x(), 10.0);
    EXPECT_EQ(projection.second.y(), arc_length - 10.0);
  }
  for (auto arc_length = 21.0; arc_length <= 30.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_EQ(projection.first.x(), 10.0 + (20 - arc_length));
    EXPECT_EQ(projection.first.y(), 10.0);
    EXPECT_EQ(projection.second.x(), 10.0 + (20 - arc_length));
    EXPECT_EQ(projection.second.y(), 10.0);
  }
}

TEST(DrivableAreaExpansionProjection, SubLinestring)
{
  using drivable_area_expansion::sub_linestring;

  const linestring_t ls = {
    point_t{0.0, 0.0}, point_t{1.0, 0.0}, point_t{2.0, 0.0}, point_t{3.0, 0.0},
    point_t{4.0, 0.0}, point_t{5.0, 0.0}, point_t{6.0, 0.0},
  };
  {
    // arc lengths equal to the original range: same linestring is returned
    const auto sub = sub_linestring(ls, 0.0, 6.0);
    ASSERT_EQ(ls.size(), sub.size());
    for (auto i = 0lu; i < ls.size(); ++i) EXPECT_TRUE(boost::geometry::equals(ls[i], sub[i]));
  }
  {
    // arc lengths equal to existing point: sublinestring with same points
    const auto sub = sub_linestring(ls, 1.0, 5.0);
    ASSERT_EQ(ls.size() - 2lu, sub.size());
    for (auto i = 0lu; i < sub.size(); ++i) EXPECT_TRUE(boost::geometry::equals(ls[i + 1], sub[i]));
  }
  {
    // arc lengths inside the original: sublinestring with some interpolated points
    const auto sub = sub_linestring(ls, 1.5, 2.5);
    ASSERT_EQ(sub.size(), 3lu);
    EXPECT_EQ(sub[0].x(), 1.5);
    EXPECT_EQ(sub[1].x(), 2.0);
    EXPECT_EQ(sub[2].x(), 2.5);
    for (const auto & p : sub) EXPECT_EQ(p.y(), 0.0);
  }
  {
    // arc length outside of the original range: first & last point are replaced by interpolations
    const auto sub = sub_linestring(ls, -0.5, 8.5);
    ASSERT_EQ(sub.size(), ls.size());
    EXPECT_EQ(sub.front().x(), -0.5);
    for (auto i = 1lu; i + 1 < ls.size(); ++i) EXPECT_TRUE(boost::geometry::equals(ls[i], sub[i]));
    EXPECT_EQ(sub.back().x(), 8.5);
    for (const auto & p : sub) EXPECT_EQ(p.y(), 0.0);
  }
}

TEST(DrivableAreaExpansionProjection, InverseProjection)
{
  using drivable_area_expansion::linestring_to_point_projection;
  using drivable_area_expansion::point_to_linestring_projection;

  linestring_t ls = {
    point_t(0.0, 0.0), point_t(10.0, 0.0), point_t(10.0, 10.0), point_t(0.0, 10.0),
    point_t(5.0, 5.0)};
  constexpr auto eps = 1e-9;

  for (auto x = 0.0; x < 10.0; x += 0.1) {
    for (auto y = 0.0; x < 10.0; x += 0.1) {
      point_t p(x, y);
      const auto projection = point_to_linestring_projection(p, ls);
      const auto inverse =
        linestring_to_point_projection(ls, projection.arc_length, projection.distance);
      EXPECT_NEAR(inverse.second.x(), p.x(), eps);
      EXPECT_NEAR(inverse.second.y(), p.y(), eps);
    }
  }
}
