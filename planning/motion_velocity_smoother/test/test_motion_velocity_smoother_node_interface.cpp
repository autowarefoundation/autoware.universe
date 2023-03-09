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

#include "motion_velocity_smoother/motion_velocity_smoother_node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningErrorMonitor, testPlanningInterface)
{
  rclcpp::init(0, nullptr);
  std::cerr << "sugahara " << __FILE__ << __LINE__  << std::endl;
  auto node_options = rclcpp::NodeOptions{};
  std::cerr << "sugahara " << __FILE__ << __LINE__  << std::endl;
  auto test_target_node =
    std::make_shared<motion_velocity_smoother::MotionVelocitySmootherNode>(node_options);
  std::cerr << "sugahara " << __LINE__  << std::endl;
  auto test_manager = std::make_shared<planning_test_manager::PlanningIntefaceTestManager>();
  std::cerr << "sugahara " << __LINE__  << std::endl;
  test_manager->setOdomTopicName("/localization/kinematic_state");
  std::cerr << "sugahara " << __LINE__  << std::endl;
  test_manager->setMaxVelocityTopicName("/planning/scenario_planning/max_velocity");
  std::cerr << "sugahara " << __LINE__  << std::endl;
  test_manager->setTFTopicName("/tf");
  std::cerr << "sugahara " << __LINE__  << std::endl;
  test_manager->setReceivedMaxVelocityTopicName("/planning/scenario_planning/trajectory");
  std::cerr << "sugahara " << __LINE__  << std::endl;
  test_manager->testNominalTrajectory(*test_target_node);
  std::cerr << "sugahara " << __LINE__  << std::endl;
  EXPECT_GE(test_manager->getReceivedMaxVelocityNum(), 1);
  // /planning/scenario_planning/current_max_velocity
  // setReceivedTrajectoryTopicName();

  // test_manager->testPlaningInterface(test_target_node);
}
