// Copyright 2023 Tier IV, Inc.
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
#include "freespace_planner/freespace_planner_node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningModuleInterfaceTest, testPlanningInterfaceWithVariousTrajectoryInput)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_utils::PlanningIntefaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};

  test_manager->declareVehicleInfoParams(node_options);

  const auto freespace_planner_dir =
    ament_index_cpp::get_package_share_directory("freespace_planner");

  node_options.arguments(
    {"--ros-args", "--params-file",
     freespace_planner_dir + "/config/freespace_planner.param.yaml"});

  auto test_target_node =
    std::make_shared<freespace_planner::FreespacePlannerNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishOccupancyGrid(
    test_target_node, "freespace_planner/input/occupancy_grid");
  test_manager->publishScenario(
    test_target_node, "freespace_planner/input/external_velocity_limit_mps");

  // set subscriber for test_target_node
  test_manager->setRouteSubscriber("freespace_planner/output/route");

  // setting topic name of subscribing topic
  test_manager->setRouteInputTopicName("freespace_planner/input/route");

  // test for normal route
  ASSERT_NO_THROW(test_manager->testWithNominalRoute(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for route with empty/one point/overlapping point
  test_manager->testWithAbnormalRoute(test_target_node);
}
