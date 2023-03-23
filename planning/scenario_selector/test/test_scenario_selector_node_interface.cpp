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
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"
#include "scenario_selector/scenario_selector_node.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningModuleInterfaceTest, testPlanningInterfaceWithVariousTrajectoryInput)
{
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  rclcpp::init(0, nullptr);

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  auto test_manager = std::make_shared<planning_test_utils::PlanningIntefaceTestManager>();

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  auto node_options = rclcpp::NodeOptions{};

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  auto test_target_node = std::make_shared<ScenarioSelectorNode>(node_options);

  // publish necessary topics from test_manager
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishOdometry(test_target_node, "input/odometry");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishAcceleration(test_target_node, "input/acceleration");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishParkingState(test_target_node, "is_parking_completed");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishTrajectory(test_target_node, "input/parking/trajectory");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishMap(test_target_node, "input/lanelet_map");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishRoute(test_target_node, "input/route");

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;


  // test_target_node → test_node_
  test_manager->setTrajectorySubscriber("output/trajectory");

  // test for normal route
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  ASSERT_NO_THROW(test_manager->testWithNominalRoute(test_target_node));


  // setting topic name of subscribing topic
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->setTrajectoryInputTopicName("input/lane_driving/trajectory");

  // test for normal trajectory
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->testWithAbnormalTrajectory(test_target_node);
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  rclcpp::shutdown();
}
