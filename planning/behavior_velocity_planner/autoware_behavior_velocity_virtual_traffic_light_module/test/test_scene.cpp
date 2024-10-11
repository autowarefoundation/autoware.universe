// Copyright 2025 TIER IV, Inc.
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


#include "../src/scene.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>


using namespace autoware::behavior_velocity_planner;

TEST(SceneTest, CreateKeyValue)
{
    auto key_value = createKeyValue("test_key", "test_value");
    EXPECT_EQ(key_value.key, "test_key");
    EXPECT_EQ(key_value.value, "test_value");
}

TEST(SceneTest, ToAutowarePoints)
{
    lanelet::ConstLineString3d line_string;
    line_string.push_back(lanelet::Point3d(1, 1.0, 2.0, 3.0));
    line_string.push_back(lanelet::Point3d(2, 4.0, 5.0, 6.0));

    auto result = toAutowarePoints(line_string);
    ASSERT_EQ(result.size(), 2);
    EXPECT_EQ(result[0].x(), 1.0);
    EXPECT_EQ(result[0].y(), 2.0);
    EXPECT_EQ(result[0].z(), 3.0);
    EXPECT_EQ(result[1].x(), 4.0);
    EXPECT_EQ(result[1].y(), 5.0);
    EXPECT_EQ(result[1].z(), 6.0);
}

TEST(SceneTest, CalcCenter)
{
    autoware::universe_utils::LineString3d line_string;
    line_string.push_back(autoware::universe_utils::Point3d(1.0, 2.0, 3.0));
    line_string.push_back(autoware::universe_utils::Point3d(4.0, 5.0, 6.0));

    auto center = calcCenter(line_string);
    EXPECT_DOUBLE_EQ(center.x(), 2.5);
    EXPECT_DOUBLE_EQ(center.y(), 3.5);
    EXPECT_DOUBLE_EQ(center.z(), 4.5);
}

TEST(SceneTest, CalcHeadPose)
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

TEST(SceneTest, ConvertToGeomPoint)
{
    autoware::universe_utils::Point3d point(1.0, 2.0, 3.0);
    auto geom_point = convertToGeomPoint(point);

    EXPECT_DOUBLE_EQ(geom_point.x, 1.0);
    EXPECT_DOUBLE_EQ(geom_point.y, 2.0);
    EXPECT_DOUBLE_EQ(geom_point.z, 0.0);  // z is not set in convertToGeomPoint
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}