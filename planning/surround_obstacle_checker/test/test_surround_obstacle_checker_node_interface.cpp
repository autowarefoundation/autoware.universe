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
#include "surround_obstacle_checker/node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningModuleInterfaceTest, testPlanningInterfaceWithVariousTrajectoryInput)
{
  using autoware_auto_planning_msgs::msg::Trajectory;
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_manager::PlanningIntefaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};

  test_manager->declareVehicleInfoParams(node_options);
  test_manager->declareNearestSearchDistanceParams(node_options);

  const auto surround_obstacle_checker_dir =
    ament_index_cpp::get_package_share_directory("surround_obstacle_checker");

  node_options.arguments(
    {"--ros-args", "--params-file",
     surround_obstacle_checker_dir + "/config/surround_obstacle_checker.param.yaml"});

  auto test_target_node =
    std::make_shared<surround_obstacle_checker::SurroundObstacleCheckerNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishMaxVelocity(
    test_target_node, "surround_obstacle_checker/input/external_velocity_limit_mps");

  // set subscriber for test_target_node
  test_manager->setTrajectorySubscriber("surround_obstacle_checker/output/trajectory");

  // setting topic name of subscribing topic
  test_manager->setTrajectoryInputTopicName("surround_obstacle_checker/input/trajectory");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(
    test_target_node, "surround_obstacle_checker/input/trajectory"));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  test_manager->testWithAbnormalTrajectory(test_target_node);
}
