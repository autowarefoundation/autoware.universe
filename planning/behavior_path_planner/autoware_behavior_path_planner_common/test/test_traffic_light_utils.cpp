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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"

#include <autoware_perception_msgs/msg/detail/traffic_light_group__struct.hpp>
#include <autoware_planning_msgs/msg/detail/lanelet_route__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Forward.h>
#include <yaml-cpp/node/node.h>

#include <limits>
#include <memory>

using autoware::behavior_path_planner::PlannerData;
using autoware::behavior_path_planner::TrafficSignalStamped;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::Pose;

class BehaviorPathPlanningTrafficLightTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planner_data_ = std::make_shared<PlannerData>();
    const auto test_data_file =
      ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner_common") +
      "/test_data/test_traffic_light.yaml";
    YAML::Node config = YAML::LoadFile(test_data_file);

    set_current_pose(config);
    set_route_handler(config);
    set_traffic_signal(config);
  }

  void set_current_pose(YAML::Node config)
  {
    const auto self_odometry =
      autoware::test_utils::parse<nav_msgs::msg::Odometry>(config["self_odometry"]);
    planner_data_->self_odometry = std::make_shared<const nav_msgs::msg::Odometry>(self_odometry);
  }

  void set_route_handler(YAML::Node config)
  {
    const auto route = autoware::test_utils::parse<LaneletRoute>(config["route"]);
    const auto intersection_map =
      autoware::test_utils::make_map_bin_msg(autoware::test_utils::get_absolute_path_to_lanelet_map(
        "autoware_test_utils", "intersection/lanelet2_map.osm"));
    planner_data_->route_handler->setMap(intersection_map);
    planner_data_->route_handler->setRoute(route);
  }

  void set_traffic_signal(YAML::Node config)
  {
    const auto traffic_light =
      autoware::test_utils::parse<TrafficLightGroupArray>(config["traffic_signal"]);
    for (const auto & signal : traffic_light.traffic_light_groups) {
      TrafficSignalStamped traffic_signal;
      traffic_signal.stamp = traffic_light.stamp;
      traffic_signal.signal = signal;
      planner_data_->traffic_light_id_map[signal.traffic_light_group_id] = traffic_signal;
    }
  }

  lanelet::ConstLanelets lanelets_;
  std::shared_ptr<PlannerData> planner_data_;
};

TEST_F(BehaviorPathPlanningTrafficLightTest, getDistanceToNextTrafficLight)
{
  using autoware::behavior_path_planner::utils::traffic_light::getDistanceToNextTrafficLight;

  {
    Pose pose = autoware::test_utils::createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    lanelet::ConstLanelets lanelets;
    EXPECT_DOUBLE_EQ(
      getDistanceToNextTrafficLight(pose, lanelets), std::numeric_limits<double>::infinity());
  }
  {
  }
}

TEST_F(BehaviorPathPlanningTrafficLightTest, calcDistanceToRedTrafficLight)
{
}
