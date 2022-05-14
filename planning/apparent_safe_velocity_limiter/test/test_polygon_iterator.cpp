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

#include "apparent_safe_velocity_limiter/polygon_iterator.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

// gtest
#include <gtest/gtest.h>

// Vector
#include <random>
#include <string>
#include <vector>

using grid_map::GridMap;
using grid_map::Index;
using grid_map::Length;
using grid_map::Polygon;
using grid_map::Position;

TEST(PolygonIterator, Dummy)
{
  std::vector<std::string> types;
  types.emplace_back("type");
  GridMap map(types);
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-1.0, 1.0));
  polygon.addVertex(Position(1.0, 1.0));
  polygon.addVertex(Position(1.0, -1.0));
  polygon.addVertex(Position(-1.0, -1.0));
  apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}
TEST(PolygonIterator, FullCover)
{
  std::vector<std::string> types;
  types.emplace_back("type");
  GridMap map(types);
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-100.0, 100.0));
  polygon.addVertex(Position(100.0, 100.0));
  polygon.addVertex(Position(100.0, -100.0));
  polygon.addVertex(Position(-100.0, -100.0));

  apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 37; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Outside)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(99.0, 101.0));
  polygon.addVertex(Position(101.0, 101.0));
  polygon.addVertex(Position(101.0, 99.0));
  polygon.addVertex(Position(99.0, 99.0));

  apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);

  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Square)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-1.0, 1.5));
  polygon.addVertex(Position(1.0, 1.5));
  polygon.addVertex(Position(1.0, -1.5));
  polygon.addVertex(Position(-1.0, -1.5));

  apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, TopLeftTriangle)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-40.1, 20.6));
  polygon.addVertex(Position(40.1, 20.4));
  polygon.addVertex(Position(-40.1, -20.6));

  apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
}

TEST(PolygonIterator, MoveMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)
  map.move(Position(2.0, 0.0));

  Polygon polygon;
  polygon.addVertex(Position(6.1, 1.6));
  polygon.addVertex(Position(0.9, 1.6));
  polygon.addVertex(Position(0.9, -1.6));
  polygon.addVertex(Position(6.1, -1.6));
  apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 4; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  for (int i = 0; i < 8; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Bench)
{
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stopwatch;
  double ctor_duration{};
  double iter_duration{};

  GridMap map({"layer"});

  Polygon base_polygon;
  base_polygon.addVertex(Position(-5.0, 5.0));
  base_polygon.addVertex(Position(0.0, 5.0));
  base_polygon.addVertex(Position(5.0, 5.0));
  base_polygon.addVertex(Position(5.0, 0.0));
  base_polygon.addVertex(Position(5.0, -5.0));
  base_polygon.addVertex(Position(0.0, -5.0));
  base_polygon.addVertex(Position(-5.0, -5.0));
  base_polygon.addVertex(Position(-5.0, 0.0));

  for (double resolution = 0.001; resolution <= 1.00; resolution *= 10) {
    map.setGeometry(Length(10.0, 10.0), resolution, Position(0.0, 0.0));  // bufferSize(8, 5)
    for (auto seed = 0; seed < 1; ++seed) {
      std::cout << seed << std::endl;
      std::random_device r;
      std::default_random_engine engine(seed);
      std::uniform_real_distribution uniform_dist(-2.50, 2.50);
      Polygon polygon;
      for (const auto & vertex : base_polygon.getVertices()) {
        polygon.addVertex(vertex + Position(uniform_dist(engine), uniform_dist(engine)));
      }
      stopwatch.tic("ctor");
      apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);
      ctor_duration += stopwatch.toc("ctor");

      while (!iterator.isPastEnd()) {
        stopwatch.tic("iter");
        ++iterator;
        iter_duration += stopwatch.toc("iter");
      }
    }
  }
  std::printf(
    "Total Runtime (constructor/increment): %2.2fms, %2.2fms\n", ctor_duration, iter_duration);
}

TEST(PolygonIterator, BenchCompare)
{
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stopwatch;
  double ctor_duration{};
  double gm_ctor_duration{};
  double iter_duration{};
  double gm_iter_duration{};

  GridMap map({"layer"});

  Polygon base_polygon;
  base_polygon.addVertex(Position(-5.0, 5.0));
  base_polygon.addVertex(Position(0.0, 5.0));
  base_polygon.addVertex(Position(5.0, 5.0));
  base_polygon.addVertex(Position(5.0, 0.0));
  base_polygon.addVertex(Position(5.0, -5.0));
  base_polygon.addVertex(Position(0.0, -5.0));
  base_polygon.addVertex(Position(-5.0, -5.0));
  base_polygon.addVertex(Position(-5.0, 0.0));

  for (double resolution = 0.01; resolution <= 1.00; resolution *= 10) {
    map.setGeometry(Length(10.0, 10.0), resolution, Position(0.0, 0.0));  // bufferSize(8, 5)
    const auto move = grid_map::Position(2.0, 2.0);
    map.move(move);
    for (auto seed = 0; seed < 100; ++seed) {
      std::cout << seed << std::endl;
      std::random_device r;
      std::default_random_engine engine(seed);
      std::uniform_real_distribution uniform_dist(-2.50, 2.50);
      Polygon polygon;
      for (const auto & vertex : base_polygon.getVertices()) {
        polygon.addVertex(vertex + Position(uniform_dist(engine), uniform_dist(engine)) + move);
      }
      stopwatch.tic("ctor");
      apparent_safe_velocity_limiter::PolygonIterator iterator(map, polygon);
      ctor_duration += stopwatch.toc("ctor");
      stopwatch.tic("gm_ctor");
      grid_map::PolygonIterator iterator_gridmap(map, polygon);
      gm_ctor_duration += stopwatch.toc("gm_ctor");

      while (!iterator.isPastEnd() && !iterator_gridmap.isPastEnd()) {
        // std::cout << (*iterator).transpose() << " " << (*iterator_gridmap).transpose() <<
        // std::endl;
        ASSERT_EQ((*iterator).x(), (*iterator_gridmap).x());
        EXPECT_EQ((*iterator).y(), (*iterator_gridmap).y());
        stopwatch.tic("iter");
        ++iterator;
        iter_duration += stopwatch.toc("iter");
        stopwatch.tic("gm_iter");
        ++iterator_gridmap;
        gm_iter_duration += stopwatch.toc("gm_iter");
      }
      EXPECT_EQ(iterator.isPastEnd(), iterator_gridmap.isPastEnd());
      while (!iterator_gridmap.isPastEnd())
        std::cout << "MISSING " << (*++iterator_gridmap).transpose() << std::endl;
      while (!iterator.isPastEnd())
        std::cout << (*++iterator).transpose() << " MISSING" << std::endl;
    }
  }
  std::printf(
    "Total Runtimes (constructor/increment):\nCustom: %2.2fms, %2.2fms\nGridMap: %2.2fms, "
    "%2.2fms\n",
    ctor_duration, gm_ctor_duration, iter_duration, gm_iter_duration);
}
