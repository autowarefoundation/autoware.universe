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
#include "behavior_velocity_planner/node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

TEST(PlanningModuleInterfaceTest, testPlanningInterfaceWithEmptyRouteInput)
{
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  rclcpp::init(0, nullptr);

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  auto test_manager = std::make_shared<planning_test_utils::PlanningIntefaceTestManager>();

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  auto node_options = rclcpp::NodeOptions{};

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->declareVehicleInfoParams(node_options);
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->declareNearestSearchDistanceParams(node_options);

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  const auto behavior_velocity_planner_dir =
    ament_index_cpp::get_package_share_directory("behavior_velocity_planner");

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  node_options.arguments(
    {"--ros-args",
     "--params-file",
     behavior_velocity_planner_dir + "/config/behavior_velocity_planner.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/blind_spot.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/crosswalk.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/detection_area.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/intersection.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/no_stopping_area.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/occlusion_spot.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/run_out.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/speed_bump.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/stop_line.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/traffic_light.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/virtual_traffic_light.param.param.yaml",
     "--params-file",
     behavior_velocity_planner_dir + "/config/out_of_lane.param.yaml"});

  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  auto test_target_node =
    std::make_shared<behavior_velocity_planner::BehaviorVelocityPlannerNode>(node_options);

  // publish necessary topics from test_manager
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishAcceleration(test_target_node, "behavior_velocity_planner/input/accel");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishPredictedObjects(
    test_target_node, "behavior_velocity_planner/input/dynamic_objects");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishPointCloud(
    test_target_node, "behavior_velocity_planner/input/no_ground_pointcloud");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishOdometry(
    test_target_node, "behavior_velocity_planner/input/vehicle_odometry");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishAcceleration(test_target_node, "behavior_velocity_planner/input/accel");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishMap(test_target_node, "behavior_velocity_planner/input/vector_map");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishTrafficSignals(
    test_target_node, "behavior_velocity_planner/input/traffic_signals");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishExternalCrosswalkStates(
    test_target_node, "behavior_velocity_planner/input/external_crosswalk_states");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishExternalIntersectionStates(
    test_target_node, "behavior_velocity_planner/input/external_intersection_states");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishMaxVelocity(
    test_target_node, "behavior_velocity_planner/input/external_velocity_limit_mps");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishExternalTrafficSignals(
    test_target_node, "behavior_velocity_planner/input/external_traffic_signals");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishVirtualTrafficLightState(
    test_target_node, "behavior_velocity_planner/input/virtual_traffic_light_states");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishTrafficSignals(
    test_target_node, "behavior_velocity_planner/input/traffic_signals");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->publishOccupancyGrid(
    test_target_node, "behavior_velocity_planner/input/occupancy_grid");
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;

  // set subscriber with topic name: behavior_velocity_planner → test_node_
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->setPathSubscriber("behavior_velocity_planner/output/path");

  // set behavior_velocity_planner node's input topic name(this topic is changed to test node):
  // test_node_ → behavior_velocity_planner
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  test_manager->setPathWithLaneIdTopicName("/behavior_velocity_planner/input/path_with_lane_id");

  // test for normal trajectory
  std::cerr << "print debug " << __FILE__ << __LINE__ << std::endl;
  ASSERT_NO_THROW(test_manager->testWithNominalPathWithLaneId(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  // test_manager->testWithAbnormalPathWithLaneId(test_target_node);
  rclcpp::shutdown();
}
