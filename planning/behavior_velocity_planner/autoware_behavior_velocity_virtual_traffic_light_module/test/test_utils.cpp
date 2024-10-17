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

#include "../src/utils.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

using autoware::behavior_velocity_planner::virtual_traffic_light::calcCenter;
using autoware::behavior_velocity_planner::virtual_traffic_light::calcHeadPose;
using autoware::behavior_velocity_planner::virtual_traffic_light::convertToGeomPoint;
using autoware::behavior_velocity_planner::virtual_traffic_light::createKeyValue;
using autoware::behavior_velocity_planner::virtual_traffic_light::insertStopVelocityAtCollision;
using autoware::behavior_velocity_planner::virtual_traffic_light::insertStopVelocityFromStart;
using autoware::behavior_velocity_planner::virtual_traffic_light::SegmentIndexWithPoint;
using autoware::behavior_velocity_planner::virtual_traffic_light::toAutowarePoints;

tier4_planning_msgs::msg::PathWithLaneId generateStraightPath()
{
  tier4_planning_msgs::msg::PathWithLaneId path;
  for (size_t i = 0; i < 10; ++i) {
    tier4_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i);
    point.point.pose.position.y = 0;
    point.point.pose.position.z = 0;
    point.point.longitudinal_velocity_mps = 10.0;
    path.points.push_back(point);
  }
  return path;
}

TEST(VirtualTrafficLightTest, CreateKeyValue)
{
  auto key_value = createKeyValue("test_key", "test_value");
  EXPECT_EQ(key_value.key, "test_key");
  EXPECT_EQ(key_value.value, "test_value");
}

TEST(VirtualTrafficLightTest, ToAutowarePoints)
{
  lanelet::LineString3d line_string;
  line_string.push_back(lanelet::Point3d(1, 1.0, 2.0, 3.0));
  line_string.push_back(lanelet::Point3d(2, 4.0, 5.0, 6.0));

  const auto result = toAutowarePoints(line_string);
  ASSERT_EQ(result.size(), 2);
  EXPECT_DOUBLE_EQ(result[0].x(), 1.0);
  EXPECT_DOUBLE_EQ(result[0].y(), 2.0);
  EXPECT_DOUBLE_EQ(result[0].z(), 3.0);
  EXPECT_DOUBLE_EQ(result[1].x(), 4.0);
  EXPECT_DOUBLE_EQ(result[1].y(), 5.0);
  EXPECT_DOUBLE_EQ(result[1].z(), 6.0);
}

TEST(VirtualTrafficLightTest, CalcCenter)
{
  autoware::universe_utils::LineString3d line_string;
  line_string.emplace_back(1.0, 2.0, 3.0);
  line_string.emplace_back(4.0, 5.0, 6.0);

  auto center = calcCenter(line_string);
  EXPECT_DOUBLE_EQ(center.x(), 2.5);
  EXPECT_DOUBLE_EQ(center.y(), 3.5);
  EXPECT_DOUBLE_EQ(center.z(), 4.5);
}

TEST(VirtualTrafficLightTest, CalcHeadPose)
{
  geometry_msgs::msg::Pose base_link_pose;
  base_link_pose.position.x = 1.0;
  base_link_pose.position.y = 2.0;
  base_link_pose.position.z = 3.0;

  double base_link_to_front = 1.0;
  auto head_pose = calcHeadPose(base_link_pose, base_link_to_front);

  EXPECT_DOUBLE_EQ(head_pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(head_pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(head_pose.position.z, 3.0);
}

TEST(VirtualTrafficLightTest, ConvertToGeomPoint)
{
  autoware::universe_utils::Point3d point(1.0, 2.0, 3.0);
  auto geom_point = convertToGeomPoint(point);

  EXPECT_DOUBLE_EQ(geom_point.x, 1.0);
  EXPECT_DOUBLE_EQ(geom_point.y, 2.0);
  EXPECT_DOUBLE_EQ(geom_point.z, 0.0);  // z is not set in convertToGeomPoint
}

TEST(VirtualTrafficLightTest, InsertStopVelocityFromStart)
{
  tier4_planning_msgs::msg::PathWithLaneId path;
  tier4_planning_msgs::msg::PathPointWithLaneId point;
  point.point.longitudinal_velocity_mps = 10.0;
  path.points.push_back(point);

  insertStopVelocityFromStart(&path);
  EXPECT_DOUBLE_EQ(path.points[0].point.longitudinal_velocity_mps, 0.0);
}

TEST(VirtualTrafficLightTest, InsertStopVelocityAtCollision)
{
  // 1) insert stop velocity at first point
  {
    std::cout << "----- insert stop velocity at first point -----" << std::endl;
    auto path = generateStraightPath();
    const double offset = 0.0;
    SegmentIndexWithPoint collision;
    collision.index = 0;
    collision.point.x = 0.0;
    collision.point.y = 0.0;
    const auto result = insertStopVelocityAtCollision(collision, offset, &path);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 0);
    const auto & point = path.points.at(result.value());
    EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, 0.0);
    // todo(someone): fix bug of arc_lane_utils::findOffsetSegment
    // EXPECT_DOUBLE_EQ(point.point.pose.position.x, 0.0 + offset);
    EXPECT_DOUBLE_EQ(point.point.pose.position.y, 0);
  }

  // 2) insert stop velocity at middle point
  {
    std::cout << "----- insert stop velocity at middle point -----" << std::endl;
    auto path = generateStraightPath();
    const double offset = 0.0;
    SegmentIndexWithPoint collision;
    collision.index = 5;
    collision.point.x = 5.0;
    collision.point.y = 0.0;
    const auto result = insertStopVelocityAtCollision(collision, offset, &path);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 5);
    const auto & point = path.points.at(result.value());
    EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, 0.0);
    // todo(someone): fix bug of arc_lane_utils::findOffsetSegment
    // EXPECT_DOUBLE_EQ(point.point.pose.position.x, 5.0 + offset);
    EXPECT_DOUBLE_EQ(point.point.pose.position.y, 0);
  }

  // 3) insert stop velocity at last
  // NOTE: autoware::motion_utils::calcLongitudinalOffsetToSegment() return std::nan("");
  //       so cannot insert stop velocity at last point.
  // todo(someone): We need to review whether this is the correct specification.
  {
    std::cout << "----- insert stop velocity at last -----" << std::endl;
    auto path = generateStraightPath();
    const double offset = 0.0;
    SegmentIndexWithPoint collision;
    collision.index = 9;
    collision.point.x = 9.0;
    collision.point.y = 0.0;
    const auto result = insertStopVelocityAtCollision(collision, offset, &path);
    ASSERT_FALSE(result.has_value());
  }

  // 4) insert stop velocity at middle point with offset 0.01
  {
    std::cout << "----- insert stop velocity at middle point with offset 0.01 -----" << std::endl;
    auto path = generateStraightPath();
    const double offset = 0.01;
    SegmentIndexWithPoint collision;
    collision.index = 5;
    collision.point.x = 5.0;
    collision.point.y = 0.0;
    const auto result = insertStopVelocityAtCollision(collision, offset, &path);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 6);
    const auto & point = path.points.at(result.value());
    EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, 0.0);
    EXPECT_DOUBLE_EQ(point.point.pose.position.x, 5.0 + offset);
    EXPECT_DOUBLE_EQ(point.point.pose.position.y, 0);
  }

  // 4) insert stop velocity at middle point with offset 0.5
  {
    std::cout << "----- insert stop velocity at middle point with offset 0.4 -----" << std::endl;
    auto path = generateStraightPath();
    const double offset = 0.4;
    SegmentIndexWithPoint collision;
    collision.index = 5;
    collision.point.x = 5.0;
    collision.point.y = 0.0;
    const auto result = insertStopVelocityAtCollision(collision, offset, &path);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 6);
    const auto & point = path.points.at(result.value());
    EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, 0.0);
    EXPECT_DOUBLE_EQ(point.point.pose.position.x, 5.0 + offset);
    EXPECT_DOUBLE_EQ(point.point.pose.position.y, 0);
  }

  // 5) insert stop velocity at middle point with offset 1.0
  {
    std::cout << "----- insert stop velocity at middle point with offset 1.0 -----" << std::endl;
    auto path = generateStraightPath();
    const double offset = 1.0;
    SegmentIndexWithPoint collision;
    collision.index = 5;
    collision.point.x = 5.0;
    collision.point.y = 0.0;
    const auto result = insertStopVelocityAtCollision(collision, offset, &path);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 6);
    const auto & point = path.points.at(result.value());
    EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, 0.0);
    // todo(someone): fix bug of arc_lane_utils::findOffsetSegment
    // EXPECT_DOUBLE_EQ(point.point.pose.position.x, 5.0 + offset);
    EXPECT_DOUBLE_EQ(point.point.pose.position.y, 0);
  }
}
