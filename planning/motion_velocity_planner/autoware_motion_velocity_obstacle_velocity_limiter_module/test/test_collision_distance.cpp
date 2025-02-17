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

#include "../src/distance.hpp"
#include "../src/obstacles.hpp"
#include "../src/types.hpp"
#include "autoware/motion_velocity_planner_common_universe/planner_data.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/io/wkt/write.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

const auto point_in_polygon = [](const auto x, const auto y, const auto & polygon) {
  return std::find_if(polygon.outer().begin(), polygon.outer().end(), [=](const auto & pt) {
           return pt.x() == x && pt.y() == y;
         }) != polygon.outer().end();
};

TEST(TestCollisionDistance, distanceToClosestCollisionParticleModel)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::CollisionChecker;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::distanceToClosestCollision;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::linestring_t;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::polygon_t;

  autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters params;
  params.model =
    autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters::PARTICLE;
  params.heading = 0.0;
  linestring_t vector = {{0.0, 0.0}, {5.0, 0.0}};
  polygon_t footprint;
  footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  autoware::motion_velocity_planner::obstacle_velocity_limiter::Obstacles obstacles;

  std::optional<double> result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(-1.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(1.0, 2.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(4.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 4.0);

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

TEST(TestCollisionDistance, distanceToClosestCollisionApproximation)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::CollisionChecker;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::distanceToClosestCollision;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::linestring_t;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::polygon_t;

  autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters params;
  params.distance_method = autoware::motion_velocity_planner::obstacle_velocity_limiter::
    ProjectionParameters::APPROXIMATION;
  params.heading = 0.0;
  linestring_t vector = {{0.0, 0.0}, {5.0, 0.0}};
  polygon_t footprint;
  footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  autoware::motion_velocity_planner::obstacle_velocity_limiter::Obstacles obstacles;

  auto EPS = 1e-2;

  std::optional<double> result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());
  // Non Value obstacles
  obstacles.points.emplace_back(-1.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(1.0, 2.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  // inside the polygon
  obstacles.points.emplace_back(4.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 4.0);

  obstacles.points.emplace_back(3.0, 0.5);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 3.04, EPS);

  obstacles.points.emplace_back(2.5, -0.75);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.61, EPS);

  obstacles.points.emplace_back(2.0, -1.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.23, EPS);

  // Change vector and footprint
  vector = linestring_t{{0.0, 0.0}, {5.0, 5.0}};
  params.heading = M_PI_4;
  footprint.outer() = {{-1.0, 1.0}, {4.0, 6.0}, {6.0, 4.0}, {1.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  obstacles.points.clear();
  obstacles.lines.clear();

  // auto EPS = 1e-3;
  obstacles.points.emplace_back(5.0, 1.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(4.0, 4.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 5.65, EPS);

  obstacles.points.emplace_back(1.0, 2.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.23, EPS);

  // Change vector (opposite direction)
  params.heading = -3 * M_PI_4;
  vector = linestring_t{{5.0, 5.0}, {0.0, 0.0}};
  obstacles.points.clear();
  obstacles.lines.clear();

  obstacles.points.emplace_back(1.0, 1.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 5.65, EPS);

  obstacles.points.emplace_back(4.0, 3.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.23, EPS);
}

TEST(TestCollisionDistance, distanceToClosestCollisionBicycleModel)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::CollisionChecker;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::distanceToClosestCollision;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::linestring_t;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::polygon_t;

  autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters params;
  params.model =
    autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters::BICYCLE;
  params.heading = 0.0;
  linestring_t vector = {{0.0, 0.0}, {5.0, 0.0}};
  polygon_t footprint;
  footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  autoware::motion_velocity_planner::obstacle_velocity_limiter::Obstacles obstacles;

  auto EPS = 1e-2;

  std::optional<double> result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(-1.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(1.0, 2.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(4.0, 0.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_FALSE(result.has_value());

  obstacles.points.emplace_back(3.0, 0.5);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 3.05, EPS);

  obstacles.points.emplace_back(2.5, -0.75);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.64, EPS);

  // Change vector and footprint
  vector = linestring_t{{0.0, 0.0}, {5.0, 5.0}};
  params.heading = M_PI_2;
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
  EXPECT_NEAR(*result, 6.28, EPS);

  obstacles.points.emplace_back(1.0, 2.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.31, EPS);

  // Change vector (opposite direction)
  params.heading = -M_PI;
  vector = linestring_t{{5.0, 5.0}, {0.0, 0.0}};
  obstacles.points.clear();
  obstacles.lines.clear();

  obstacles.points.emplace_back(1.0, 1.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 6.28, EPS);

  obstacles.points.emplace_back(4.0, 3.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.76, EPS);

  // change vector and footprint
  params.heading = M_PI_2;
  vector = linestring_t{{3.0, 3.0}, {0.0, 3.0}};
  footprint.outer() = {{1.0, -1.0}, {-4.0, 6.0}, {-5.0, -4.0}, {1.0, -4.0}};
  boost::geometry::correct(footprint);
  obstacles.points.clear();
  obstacles.lines.clear();

  obstacles.points.emplace_back(-2.0, -1.0);
  result =
    distanceToClosestCollision(vector, footprint, CollisionChecker(obstacles, 0lu, 0lu), params);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 7.34, EPS);
}

TEST(TestCollisionDistance, arcDistance)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::arcDistance;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::point_t;

  auto EPS = 1e-2;

  EXPECT_NEAR(arcDistance({0, 0}, M_PI_2, {1, 1}), M_PI_2, EPS);
  EXPECT_NEAR(arcDistance({0, 0}, M_PI_2, {-1, -1}), M_PI_2, EPS);
  EXPECT_NEAR(arcDistance({0, 0}, 0, {1, 1}), M_PI_2, EPS);
  EXPECT_NEAR(arcDistance({0, 0}, 0, {0, 1}), M_PI_2, EPS);
  EXPECT_NEAR(arcDistance({0, 0}, 0, {0, -1}), M_PI_2, EPS);
  EXPECT_NEAR(arcDistance({0, 0}, 0, {1, 0.5}), 1.15, EPS);
  EXPECT_NEAR(arcDistance({0, 0}, 0, {0.1, 0.5}), 0.71, EPS);
  EXPECT_NEAR(arcDistance({0, 0.2}, 0.463646716, {0.4, 0.2}), 0.41, EPS);
  EXPECT_NEAR(arcDistance({0, 0.0}, -M_PI_4, {1.0, 0.0}), 1.11, EPS);
  EXPECT_NEAR(arcDistance({1, 2.0}, -M_PI_2, {0.0, 1.0}), M_PI_2, EPS);
  EXPECT_NEAR(arcDistance({-0.6, -0.4}, -M_PI_4, {0.4, 0.2}), 1.59, EPS);
  // Edge cases: target "behind" the origin leads to a reverse distance
  EXPECT_NEAR(arcDistance({0, 0}, 0, {-1, -1}), M_PI_2, EPS);
}

TEST(TestCollisionDistance, createObjPolygons)
{
  using autoware::motion_velocity_planner::PlannerData;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::createObjectPolygons;
  using autoware_perception_msgs::msg::PredictedObject;
  using autoware_perception_msgs::msg::PredictedObjects;

  std::vector<std::shared_ptr<PlannerData::Object>> objects;

  auto polygons = createObjectPolygons(objects, 0.0, 0.0);
  EXPECT_TRUE(polygons.empty());

  PredictedObject object1;
  object1.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  object1.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  object1.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(0.0);
  object1.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;
  object1.shape.dimensions.x = 1.0;
  object1.shape.dimensions.y = 1.0;
  objects.push_back(std::make_shared<PlannerData::Object>(object1));

  polygons = createObjectPolygons(objects, 0.0, 1.0);
  EXPECT_TRUE(polygons.empty());

  polygons = createObjectPolygons(objects, 0.0, 0.0);
  ASSERT_EQ(polygons.size(), 1ul);
  EXPECT_TRUE(point_in_polygon(0.5, 0.5, polygons[0]));
  EXPECT_TRUE(point_in_polygon(0.5, -0.5, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.5, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-0.5, -0.5, polygons[0]));

  polygons = createObjectPolygons(objects, 1.0, 0.0);
  ASSERT_EQ(polygons.size(), 1ul);
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-1.0, 1.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(-1.0, -1.0, polygons[0]));

  PredictedObject object2;
  object2.kinematics.initial_pose_with_covariance.pose.position.x = 10.0;
  object2.kinematics.initial_pose_with_covariance.pose.position.y = 10.0;
  object2.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(M_PI_2);
  object2.kinematics.initial_twist_with_covariance.twist.linear.x = 2.0;
  object2.shape.dimensions.x = 2.0;
  object2.shape.dimensions.y = 1.0;
  objects.push_back(std::make_shared<PlannerData::Object>(object2));

  polygons = createObjectPolygons(objects, 0.0, 2.0);
  ASSERT_EQ(polygons.size(), 1ul);
  EXPECT_TRUE(point_in_polygon(10.5, 11.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(10.5, 9.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(9.5, 11.0, polygons[0]));
  EXPECT_TRUE(point_in_polygon(9.5, 9.0, polygons[0]));

  polygons = createObjectPolygons(objects, 0.0, 0.0);
  EXPECT_EQ(polygons.size(), 2ul);
}
