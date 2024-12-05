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

#include <autoware/behavior_velocity_blind_spot_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>

#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>

#include <gtest/gtest.h>

using autoware::test_utils::parse;

class TestWithAdjLaneData : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_data_file =
      ament_index_cpp::get_package_share_directory("autoware_behavior_velocity_blind_spot_module") +
      "/test_data/object_on_adj_lane.yaml";
    const auto config = YAML::LoadFile(test_data_file);
    const auto route = parse<autoware_planning_msgs::msg::LaneletRoute>(config["route"]);
    const auto map_path =
      autoware::test_utils::resolve_pkg_share_uri(config["map_path_uri"].as<std::string>());
    if (!map_path) {
      ASSERT_DEATH({ assert(false); }, "invalid map path");
    }
    const auto intersection_map = autoware::test_utils::make_map_bin_msg(map_path.value());
    route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
    route_handler->setMap(intersection_map);
    route_handler->setRoute(route);
    self_odometry = autoware::test_utils::create_const_shared_ptr(
      parse<nav_msgs::msg::Odometry>(config["self_odometry"]));
    dynamic_object = autoware::test_utils::create_const_shared_ptr(
      parse<autoware_perception_msgs::msg::PredictedObjects>(config["dynamic_object"]));
  }

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler{};
  std::shared_ptr<const nav_msgs::msg::Odometry> self_odometry{};
  std::shared_ptr<const autoware_perception_msgs::msg::PredictedObjects> dynamic_object{};
  const lanelet::Id lane_id_{2200};
};

TEST_F(TestWithAdjLaneData, getSiblingStraightLanelet)
{
  const auto sibling_straight_lanelet_opt =
    autoware::behavior_velocity_planner::getSiblingStraightLanelet(
      route_handler->getLaneletMapPtr()->laneletLayer.get(lane_id_),
      route_handler->getRoutingGraphPtr());
  ASSERT_NO_FATAL_FAILURE({ ASSERT_TRUE(sibling_straight_lanelet_opt.has_value()); });
  EXPECT_EQ(sibling_straight_lanelet_opt.value().id(), 2100);
}
