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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behavior_path_planner/behavior_path_planner_node.hpp"
#include "behavior_path_planner/module_status.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

using behavior_path_planner::BehaviorPathPlannerNode;
using planning_test_utils::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: behavior_path_planner → test_node_
  test_manager->setPathWithLaneIdSubscriber("behavior_path_planner/output/path");

  // set behavior_path_planner's input topic name(this topic is changed to test node)
  test_manager->setRouteInputTopicName("behavior_path_planner/input/route");

  test_manager->setInitialPoseTopicName("behavior_path_planner/input/odometry");

  return test_manager;
}

std::shared_ptr<BehaviorPathPlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto behavior_path_planner_dir =
    ament_index_cpp::get_package_share_directory("behavior_path_planner");

  node_options.append_parameter_override(
    "bt_tree_config_path", behavior_path_planner_dir + "/config/behavior_path_planner_tree.xml");

  test_utils::updateNodeOptions(
    node_options,
    {planning_test_utils_dir + "/config/test_common.param.yaml",
     planning_test_utils_dir + "/config/test_nearest_search.param.yaml",
     planning_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     behavior_path_planner_dir + "/config/behavior_path_planner.param.yaml",
     behavior_path_planner_dir + "/config/drivable_area_expansion.param.yaml",
     behavior_path_planner_dir + "/config/scene_module_manager.param.yaml",
     behavior_path_planner_dir + "/config/avoidance/avoidance.param.yaml",
     behavior_path_planner_dir + "/config/dynamic_avoidance/dynamic_avoidance.param.yaml",
     behavior_path_planner_dir + "/config/lane_change/lane_change.param.yaml",
     behavior_path_planner_dir + "/config/pull_out/pull_out.param.yaml",
     behavior_path_planner_dir + "/config/goal_planner/goal_planner.param.yaml",
     behavior_path_planner_dir + "/config/avoidance_by_lc/avoidance_by_lc.param.yaml",
     behavior_path_planner_dir + "/config/side_shift/side_shift.param.yaml"});

  return std::make_shared<BehaviorPathPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<BehaviorPathPlannerNode> test_target_node, std::string module_name = "")
{
  // publish necessary topics from test_manager
  test_manager->publishInitialPose(
    test_target_node, "behavior_path_planner/input/odometry", 0.0, module_name);
  test_manager->publishAcceleration(test_target_node, "behavior_path_planner/input/accel");
  test_manager->publishPredictedObjects(test_target_node, "behavior_path_planner/input/perception");
  test_manager->publishOccupancyGrid(
    test_target_node, "behavior_path_planner/input/occupancy_grid_map");
  test_manager->publishLaneDrivingScenario(
    test_target_node, "behavior_path_planner/input/scenario");
  test_manager->publishMap(test_target_node, "behavior_path_planner/input/vector_map");
  test_manager->publishCostMap(test_target_node, "behavior_path_planner/input/costmap");
  test_manager->publishOperationModeState(test_target_node, "system/operation_mode/state");
  test_manager->publishLateralOffset(
    test_target_node, "behavior_path_planner/input/lateral_offset");
}

bool isExpectedModuleRunning(
  std::shared_ptr<BehaviorPathPlannerNode> test_target_node, std::string module_name)
{
  // check if the expected module is working
  auto execution_requested_modules_name = test_target_node->getWaitingApprovalModules();
  for (const auto & execution_requested_module_name : execution_requested_modules_name) {
    if (execution_requested_module_name == module_name) {
      return true;
    }
  }

  return false;
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionRoute)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithBehaviorNominalRoute(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty route
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithAbnormalRoute(test_target_node));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();
  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithBehaviorNominalRoute(test_target_node));

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testRouteWithInvalidEgoPose(test_target_node));

  rclcpp::shutdown();
}

// ----- test with pull out module -----

TEST(PlanningModuleInterfaceTest, NodeTestPullOutModuleWithExceptionRoute)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  std::string module_name = "PullOut";
  publishMandatoryTopics(test_manager, test_target_node, module_name);
  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNominalRoute(test_target_node, module_name));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);
#ifndef USE_OLD_ARCHITECTURE
  module_name = "pull_out";
#endif
  EXPECT_EQ(isExpectedModuleRunning(test_target_node, module_name), true);

  // test with empty route
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithAbnormalRoute(test_target_node));
  rclcpp::shutdown();
}
