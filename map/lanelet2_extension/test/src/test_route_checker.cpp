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

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/route_checker.hpp"

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

#include <gtest/gtest.h>
#include <stdio.h>

#include <memory>
#include <vector>

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::utils::getId;

class TestSuite : public ::testing::Test
{
public:
  TestSuite() : sample_map_ptr(new lanelet::LaneletMap())
  {
    // NOLINT
    // create sample lanelets
    Point3d p1, p2, p3, p4;

    p1 = Point3d(getId(), 0., 0., 0.);
    p2 = Point3d(getId(), 0., 1., 0.);

    LineString3d ls_left(getId(), {p1, p2});  // NOLINT

    p3 = Point3d(getId(), 1., 0., 0.);
    p4 = Point3d(getId(), 1., 1., 0.);

    LineString3d ls_right(getId(), {p3, p4});  // NOLINT

    Lanelet lanelet(getId(), ls_left, ls_right);

    sample_map_ptr->add(lanelet);

    // create sample routes
    autoware_auto_mapping_msgs::msg::MapPrimitive map_primitive;
    autoware_auto_mapping_msgs::msg::HADMapSegment map_segment1;
    autoware_auto_mapping_msgs::msg::HADMapSegment map_segment2;

    for (size_t i = 0; i < 2; i++) {
      map_primitive.id = lanelet.id();
      map_segment1.primitives.push_back(map_primitive);
      map_primitive.id = ls_left.id();
      map_segment2.primitives.push_back(map_primitive);
    }
    sample_route1.segments.push_back(map_segment1);
    sample_route2.segments.push_back(map_segment2);
  }
  ~TestSuite() {}

  lanelet::LaneletMapPtr sample_map_ptr;
  autoware_auto_planning_msgs::msg::HADMapRoute sample_route1;  // valid route
  autoware_auto_planning_msgs::msg::HADMapRoute sample_route2;  // invalid route

private:
};

TEST_F(TestSuite, isRouteValid)
{
  autoware_auto_mapping_msgs::msg::HADMapBin bin_msg;

  autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr route_ptr1;
  route_ptr1 = std::make_shared<autoware_auto_planning_msgs::msg::HADMapRoute>(sample_route1);
  autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr route_ptr2;
  route_ptr2 = std::make_shared<autoware_auto_planning_msgs::msg::HADMapRoute>(sample_route2);

  // toBinMsg is tested at test_message_conversion.cpp
  lanelet::utils::conversion::toBinMsg(sample_map_ptr, &bin_msg);
  autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_ptr;
  map_ptr = std::make_shared<autoware_auto_mapping_msgs::msg::HADMapBin>(bin_msg);

  ASSERT_TRUE(lanelet::utils::route::isRouteValid(route_ptr1, map_ptr))
    << "The route should be valid, which should be created on the same map as the current one";
  ASSERT_FALSE(lanelet::utils::route::isRouteValid(route_ptr2, map_ptr))
    << "The route should be invalid, which should be created on the different map from the current "
       "one";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
