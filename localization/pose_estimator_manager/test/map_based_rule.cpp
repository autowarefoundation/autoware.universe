// Copyright 2023 Autoware Foundation
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

#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

#include "pose_estimator_manager/rule_helper/eagleye_area.hpp"
#include "pose_estimator_manager/rule_helper/grid_info.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <unordered_set>

lanelet::Polygon3d create_polygon3d()
{
  lanelet::Polygon3d polygon;
  lanelet::Attribute attribute("eagleye_area");
  polygon.setAttribute(lanelet::AttributeName::Type, attribute);
  lanelet::Id index = 0;
  polygon.push_back(lanelet::Point3d(index++, 0, 0));
  polygon.push_back(lanelet::Point3d(index++, 10, 0));
  polygon.push_back(lanelet::Point3d(index++, 10, 10));
  polygon.push_back(lanelet::Point3d(index++, 0, 10));
  return polygon;
}

TEST(MapBasedRule, eagleyeArea)
{
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;

  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet_map->add(create_polygon3d());

  HADMapBin msg;
  lanelet::utils::conversion::toBinMsg(lanelet_map, &msg);

  using Point = geometry_msgs::msg::Point;

  multi_pose_estimator::EagleyeArea eagleye_area;
  eagleye_area.init(std::make_shared<HADMapBin>(msg));

  EXPECT_TRUE(eagleye_area.within(Point().set__x(5).set__y(5).set__z(0)));
  EXPECT_FALSE(eagleye_area.within(Point().set__x(-5).set__y(-5).set__z(0)));
  EXPECT_EQ(eagleye_area.debug_string(), "0,0 10,0 10,10 0,10 0,0 \n");
}

TEST(MapBasedRule, gridInfo)
{
  using multi_pose_estimator::GridInfo;
  EXPECT_TRUE(GridInfo(10., -5.) == GridInfo(10., -10.));
  EXPECT_TRUE(GridInfo(10., -5.) != GridInfo(10., -15.));

  EXPECT_TRUE(GridInfo(10., -5.).get_center_point().x == 15.f);
  EXPECT_TRUE(GridInfo(10., -5.).get_center_point().y == -5.f);
  EXPECT_TRUE(GridInfo(10., -5.).get_center_point().z == 0.f);

  std::unordered_set<GridInfo> set;
  set.emplace(10., -5.);
  EXPECT_EQ(set.count(GridInfo(10., -5.)), 1ul);
  EXPECT_EQ(set.count(GridInfo(10., -15.)), 0ul);
}
