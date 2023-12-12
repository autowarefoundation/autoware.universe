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

#include "pose_estimator_arbiter/switch_rule/map_based_rule.hpp"

#include "pose_estimator_arbiter/rule_helper/eagleye_area.hpp"
#include "pose_estimator_arbiter/rule_helper/grid_info.hpp"
#include "pose_estimator_arbiter/rule_helper/pcd_occupancy.hpp"

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

class MapBasedRule : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  std::shared_ptr<rclcpp::Node> node{nullptr};

  virtual void TearDown() { rclcpp::shutdown(); }
};

TEST_F(MapBasedRule, eagleyeArea)
{
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;

  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet_map->add(create_polygon3d());

  HADMapBin msg;
  lanelet::utils::conversion::toBinMsg(lanelet_map, &msg);

  using Point = geometry_msgs::msg::Point;

  pose_estimator_arbiter::rule_helper::EagleyeArea eagleye_area(&(*node));
  eagleye_area.init(std::make_shared<HADMapBin>(msg));

  EXPECT_TRUE(eagleye_area.within(Point().set__x(5).set__y(5).set__z(0)));
  EXPECT_FALSE(eagleye_area.within(Point().set__x(-5).set__y(-5).set__z(0)));
  EXPECT_EQ(eagleye_area.debug_string(), "0,0 10,0 10,10 0,10 0,0 \n");
}

TEST_F(MapBasedRule, pcdOccupancy)
{
  using pose_estimator_arbiter::rule_helper::PcdOccupancy;
  node->declare_parameter<int>("pcd_occupancy_rule/pcd_density_upper_threshold", 20);
  node->declare_parameter<int>("pcd_occupancy_rule/pcd_density_lower_threshold", 10);

  pose_estimator_arbiter::rule_helper::PcdOccupancy pcd_occupancy(&(*node));
  geometry_msgs::msg::Point point;
  std::string message;

  // Since we have not yet given a point cloud, this returns false.
  EXPECT_FALSE(pcd_occupancy.ndt_can_operate(point, &message));
}

TEST_F(MapBasedRule, gridInfo)
{
  using pose_estimator_arbiter::rule_helper::GridInfo;
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
