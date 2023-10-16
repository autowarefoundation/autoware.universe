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

#include "obstacle_velocity_limiter/distance.hpp"
#include "obstacle_velocity_limiter/obstacles.hpp"
#include "obstacle_velocity_limiter/types.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/io/wkt/write.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include<iostream>

const auto point_in_polygon = [](const auto x, const auto y, const auto & polygon) {
  return std::find_if(polygon.outer().begin(), polygon.outer().end(), [=](const auto & pt) {
           return pt.x() == x && pt.y() == y;
         }) != polygon.outer().end();
};

TEST(TestCollisionDistance, distanceToClosestCollision)
{
  using obstacle_velocity_limiter::CollisionChecker;
  using obstacle_velocity_limiter::distanceToClosestCollision;
  using obstacle_velocity_limiter::linestring_t;
  using obstacle_velocity_limiter::polygon_t;

  obstacle_velocity_limiter::ProjectionParameters params;
  params.model = obstacle_velocity_limiter::ProjectionParameters::PARTICLE;
  params.heading = 0.0;
  linestring_t vector = {{0.0, 0.0}, {5.0, 0.0}};
  polygon_t footprint;
  footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  obstacle_velocity_limiter::Obstacles obstacles;

  std::optional<double> result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(-1.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(1.5, 2.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

//this one V

  obstacles.points.emplace_back(4.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 4.0);
    // printf("particle: %f", *result);

  obstacles.points.emplace_back(3.0, 0.5);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 3.0);

  obstacles.points.emplace_back(2.75, -0.75);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 2.75);
    // printf("result in 87: %f", *result);

  // Change vector and footprint
  vector = linestring_t{{0.0, 0.0}, {5.0, 5.0}};
  params.heading = M_PI_4;
  footprint.outer() = {{-1.0, 1.0}, {4.0, 6.0}, {6.0, 4.0}, {1.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  obstacles.points.clear();
  obstacles.lines.clear();

  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(4.0, 4.0);
  
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, std::sqrt(2 * 4.0 * 4.0));

  obstacles.points.emplace_back(1.0, 2.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.121, 1e-3);

//this one V
  obstacles.lines.push_back(linestring_t{{-2.0, 2.0}, {3.0, -1.0}});
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.354, 1e-3);

  obstacles.lines.push_back(linestring_t{{-1.5, 1.5}, {0.0, 0.5}});
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.141, 1e-3);

  obstacles.lines.push_back(linestring_t{{0.5, 1.0}, {0.5, -0.5}});
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.0, 1e-3);

  obstacles.points.clear();
  obstacles.lines.clear();
  obstacles.lines.push_back(linestring_t{{0.5, 1.0}, {0.5, 0.0}, {1.5, 0.0}});
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.353, 1e-3);

  // Change vector (opposite direction)
  params.heading = -3 * M_PI_4;
  vector = linestring_t{{5.0, 5.0}, {0.0, 0.0}};
  obstacles.points.clear();
  obstacles.lines.clear();

  obstacles.points.emplace_back(1.0, 1.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, std::sqrt(2 * 4.0 * 4.0));

  obstacles.points.emplace_back(4.0, 3.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.121, 1e-3);
}

// TEST(BicycleModel, distanceToClosestCollision)
// {

//   using obstacle_velocity_limiter::CollisionChecker;
//   using obstacle_velocity_limiter::distanceToClosestCollision;
//   using obstacle_velocity_limiter::linestring_t;
//   using obstacle_velocity_limiter::polygon_t;

//   obstacle_velocity_limiter::ProjectionParameters params;
//   params.model = obstacle_velocity_limiter::ProjectionParameters::BICYCLE;
//   params.heading = 0.0;
//   linestring_t vector = {{0.0, 0.0}, {5.0, 0.0}};
//   polygon_t footprint;
//   footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
//   boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
//   obstacle_velocity_limiter::Obstacles obstacles;


//   obstacles.points.emplace_back(4.0, 0.0);
//   std::optional<double> result =
//     distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
//   printf("bicycle : %f", *result);
//   ASSERT_TRUE(result.has_value());
//   EXPECT_DOUBLE_EQ(*result, 4.0);
    

// //----------------------------------------------------------------------------------
//   vector = linestring_t{{0.0, 0.0}, {5.0, 5.0}};
//   params.heading = M_PI_4;
//   footprint.outer() = {{-1.0, 1.0}, {4.0, 6.0}, {6.0, 4.0}, {1.0, -1.0}};
//   boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
//   obstacles.points.clear();
//   obstacles.lines.clear();


//     obstacles.lines.push_back(linestring_t{{-2.0, 2.0}, {3.0, -1.0}});
//   result =
//     distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
//   ASSERT_TRUE(result.has_value());
//   EXPECT_NEAR(*result, 0.354, 1e-3);
//   printf("result: %f", *result);

//   obstacle_velocity_limiter::ProjectionParameters params;
//   params.model = obstacle_velocity_limiter::ProjectionParameters::BICYCLE;
//   using obstacle_velocity_limiter::arcDistance;
//   using obstacle_velocity_limiter::point_t;

//   point_t projection = {1.0, 0.0};
//   double heading = M_PI/4;
//   point_t obs_point = {2.0, 1.0};

//   std::optional<double> result = arcDistance(projection, heading, obs_point);
//   ASSERT_TRUE(result.has_value());
//   EXPECT_DOUBLE_EQ(*result, 0.0);

//   result = arcDistance({1.0, 0.0}, M_PI, {2.0, 1.0});
//   ASSERT_TRUE(result.has_value());
//   EXPECT_DOUBLE_EQ(*result, 1.570);

//   result = arcDistance({1.0, 2.0}, M_PI/2, {3.0, 1.5});
//   ASSERT_TRUE(result.has_value());
//   EXPECT_DOUBLE_EQ(*result, 2.449);

//   result = arcDistance({1.0, 2.0}, 0, {3.0, 1.5});
//   ASSERT_TRUE(result.has_value());
//   EXPECT_NEAR(*result, 2.082, 1e-3);








// }

TEST(ApproximationMethod, distanceToClosestCollision)
{
  using obstacle_velocity_limiter::CollisionChecker;
  using obstacle_velocity_limiter::distanceToClosestCollision;
  // using obstacle_velocity_limiter::distance;
  using obstacle_velocity_limiter::CollisionChecker;
  using obstacle_velocity_limiter::linestring_t;
  using obstacle_velocity_limiter::polygon_t;
  using obstacle_velocity_limiter::point_t;



  obstacle_velocity_limiter::ProjectionParameters params;
  params.distance_method = obstacle_velocity_limiter::ProjectionParameters::APPROXIMATION;
  params.heading = 0.0;
  linestring_t vector = {{0.0, 0.0}, {5.0, 0.0}};
  polygon_t footprint;
  footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  obstacle_velocity_limiter::Obstacles obstacles;


  obstacles.points.emplace_back(4.0, 0.0);
  std::optional<double> result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  printf("approximation : %f", *result);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 4.0);
    

//----------------------------------------------------------------------------------
  vector = linestring_t{{0.0, 0.0}, {5.0, 5.0}};
  params.heading = M_PI_4;
  footprint.outer() = {{-1.0, 1.0}, {4.0, 6.0}, {6.0, 4.0}, {1.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  obstacles.points.clear();
  obstacles.lines.clear();


    obstacles.lines.push_back(linestring_t{{-2.0, 2.0}, {3.0, -1.0}});
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.354, 1e-3);

}



