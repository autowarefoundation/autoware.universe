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
#include "motion_velocity_smoother/motion_velocity_smoother_node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningErrorMonitor, testPlanningInterface)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_manager::PlanningIntefaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};

  test_manager->declareVehicleInfoParams(node_options);
  test_manager->declareNearestSearchDistanceParams(node_options);
  node_options.append_parameter_override("algorithm_type", "JerkFiltered");
  node_options.append_parameter_override("publish_debug_trajs", false);

  const auto motion_velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("motion_velocity_smoother");
  node_options.arguments(
    {"--ros-args", "--params-file",
     motion_velocity_smoother_dir + "/config/default_motion_velocity_smoother.param.yaml",
     "--params-file", motion_velocity_smoother_dir + "/config/default_common.param.yaml",
     "--params-file", motion_velocity_smoother_dir + "/config/JerkFiltered.param.yaml"});
  auto test_target_node =
    std::make_shared<motion_velocity_smoother::MotionVelocitySmootherNode>(node_options);
  test_manager->setOdomTopicName("/localization/kinematic_state");
  test_manager->setMaxVelocityTopicName("/planning/scenario_planning/max_velocity");
  test_manager->setTFTopicName("/tf");
  test_manager->setReceivedMaxVelocityTopicName("/planning/scenario_planning/trajectory");
  test_manager->testNominalTrajectory(*test_target_node);
  EXPECT_GE(test_manager->getReceivedMaxVelocityNum(), 1);
  // /planning/scenario_planning/current_max_velocity
  // setReceivedTrajectoryTopicName();

  // test_manager->testPlaningInterface(test_target_node);
}
