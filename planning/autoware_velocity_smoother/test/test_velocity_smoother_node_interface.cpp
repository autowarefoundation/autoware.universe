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

#include "autoware/velocity_smoother/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::planning_test_manager::PlanningInterfaceTestManager;
using autoware::velocity_smoother::VelocitySmootherNode;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();
  test_manager->setTrajectorySubscriber("velocity_smoother/output/trajectory");
  test_manager->setTrajectoryInputTopicName("velocity_smoother/input/trajectory");
  test_manager->setOdometryTopicName("/localization/kinematic_state");
  return test_manager;
}

std::shared_ptr<VelocitySmootherNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("algorithm_type", "JerkFiltered");
  node_options.append_parameter_override("publish_debug_trajs", false);
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");
  node_options.arguments(
    {"--ros-args", "--params-file", autoware_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file", velocity_smoother_dir + "/config/default_velocity_smoother.param.yaml",
     "--params-file", velocity_smoother_dir + "/config/default_common.param.yaml", "--params-file",
     velocity_smoother_dir + "/config/JerkFiltered.param.yaml"});

  return std::make_shared<VelocitySmootherNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  rclcpp::Node::SharedPtr test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishMaxVelocity(
    test_target_node, "velocity_smoother/input/external_velocity_limit_mps");
  test_manager->publishOperationModeState(
    test_target_node, "velocity_smoother/input/operation_mode_state");
  test_manager->publishAcceleration(test_target_node, "velocity_smoother/input/acceleration");
}

TEST(PlanningModuleInterfaceTest, testPlanningInterfaceWithVariousTrajectoryInput)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(test_target_node));

  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW(test_manager->testTrajectoryWithInvalidEgoPose(test_target_node));

  rclcpp::shutdown();
}
