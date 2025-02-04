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

#include "node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <tier4_planning_msgs/msg/expand_stop_range.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::motion_planning::ObstacleStopPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;
using tier4_planning_msgs::msg::ExpandStopRange;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();
  test_manager->subscribeOutput<autoware_planning_msgs::msg::Trajectory>(
    "obstacle_stop_planner/output/trajectory");
  return test_manager;
}

std::shared_ptr<ObstacleStopPlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto obstacle_stop_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_obstacle_stop_planner");

  node_options.append_parameter_override("enable_slow_down", false);

  node_options.arguments(
    {"--ros-args", "--params-file", autoware_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file", obstacle_stop_planner_dir + "/config/common.param.yaml", "--params-file",
     obstacle_stop_planner_dir + "/config/adaptive_cruise_control.param.yaml", "--params-file",
     obstacle_stop_planner_dir + "/config/obstacle_stop_planner.param.yaml"});

  return std::make_shared<ObstacleStopPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<ObstacleStopPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "obstacle_stop_planner/input/odometry", autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    test_target_node, "obstacle_stop_planner/input/pointcloud",
    sensor_msgs::msg::PointCloud2{}.set__header(
      std_msgs::msg::Header{}.set__frame_id("base_link")));
  test_manager->publishInput(
    test_target_node, "obstacle_stop_planner/input/acceleration",
    geometry_msgs::msg::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    test_target_node, "obstacle_stop_planner/input/objects",
    autoware_perception_msgs::msg::PredictedObjects{});
  test_manager->publishInput(
    test_target_node, "obstacle_stop_planner/input/expand_stop_range",
    tier4_planning_msgs::msg::ExpandStopRange{});
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "obstacle_stop_planner/input/trajectory";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithNormalTrajectory(test_target_node, input_trajectory_topic));

  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  test_manager->testWithAbnormalTrajectory(test_target_node, input_trajectory_topic);

  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "obstacle_stop_planner/input/trajectory";
  const std::string input_odometry_topic = "obstacle_stop_planner/input/odometry";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithNormalTrajectory(test_target_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithOffTrackOdometry(test_target_node, input_odometry_topic));

  rclcpp::shutdown();
}
