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

#include "behavior_path_planner/utils/safety_check.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

constexpr double epsilon = 1e-6;

using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

TEST(BehaviorPathPlanningSafetyUtilsTest, createExtendedEgoPolygon)
{
  using behavior_path_planner::utils::safety_check::createExtendedEgoPolygon;

  vehicle_info_util::VehicleInfo vehicle_info;
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.vehicle_width_m = 2.0;
  vehicle_info.rear_overhang_m = 1.0;

  {
    Pose ego_pose;
    ego_pose.position = tier4_autoware_utils::createPoint(0.0, 0.0, 0.0);
    ego_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(0.0);

    const double lon_length = 10.0;
    const double lat_margin = 2.0;

    const auto ego_polygon =
      createExtendedEgoPolygon(ego_pose, vehicle_info, lon_length, lat_margin);

    EXPECT_EQ(ego_polygon.outer().size(), static_cast<unsigned int>(5));

    const auto p1 = ego_polygon.outer().at(0);
    const auto p2 = ego_polygon.outer().at(1);
    const auto p3 = ego_polygon.outer().at(2);
    const auto p4 = ego_polygon.outer().at(3);
    EXPECT_NEAR(p1.x(), 14.0, epsilon);
    EXPECT_NEAR(p1.y(), 3.0, epsilon);
    EXPECT_NEAR(p2.x(), 14.0, epsilon);
    EXPECT_NEAR(p2.y(), -3.0, epsilon);
    EXPECT_NEAR(p3.x(), -1.0, epsilon);
    EXPECT_NEAR(p3.y(), -3.0, epsilon);
    EXPECT_NEAR(p4.x(), -1.0, epsilon);
    EXPECT_NEAR(p4.y(), 3.0, epsilon);
  }
}
