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

#include "autoware_planning_test_manager/autoware_planning_test_manager.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace autoware::planning_test_manager
{
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;
using tier4_planning_msgs::msg::PathWithLaneId;

PlanningInterfaceTestManager::PlanningInterfaceTestManager()
{
  test_node_ = std::make_shared<rclcpp::Node>("planning_interface_test_node");
}

void PlanningInterfaceTestManager::testWithNormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  publishInput(
    target_node, topic_name, autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0), 5);
}

void PlanningInterfaceTestManager::testWithAbnormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  publishInput(target_node, topic_name, Trajectory{}, 5);
  publishInput(
    target_node, topic_name, autoware::test_utils::generateTrajectory<Trajectory>(1, 0.0), 5);
  publishInput(
    target_node, topic_name,
    autoware::test_utils::generateTrajectory<Trajectory>(10, 0.0, 0.0, 0.0, 0.0, 1), 5);
}

void PlanningInterfaceTestManager::testWithNormalRoute(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  publishInput(target_node, topic_name, autoware::test_utils::makeNormalRoute(), 5);
}

void PlanningInterfaceTestManager::testWithAbnormalRoute(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  publishInput(target_node, topic_name, LaneletRoute{}, 5);
}

void PlanningInterfaceTestManager::testWithBehaviorNormalRoute(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  publishInput(target_node, topic_name, autoware::test_utils::makeBehaviorNormalRoute(), 5);
}

void PlanningInterfaceTestManager::testWithNormalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  try {
    const auto path = autoware::test_utils::loadPathWithLaneIdInYaml();
    publishInput(target_node, topic_name, path, 5);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
}

void PlanningInterfaceTestManager::testWithAbnormalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  publishInput(target_node, topic_name, PathWithLaneId{}, 5);
}

void PlanningInterfaceTestManager::testWithNormalPath(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  try {
    const auto path = autoware::test_utils::loadPathWithLaneIdInYaml();
    publishInput(
      target_node, topic_name,
      autoware::motion_utils::convertToPath<tier4_planning_msgs::msg::PathWithLaneId>(path), 5);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
}

void PlanningInterfaceTestManager::testWithAbnormalPath(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  publishInput(target_node, topic_name, Path{}, 5);
}

void PlanningInterfaceTestManager::testWithOffTrackInitialPoses(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  for (const auto & deviation : {0.0, 1.0, 10.0, 100.0}) {
    publishInput(target_node, topic_name, autoware::test_utils::makeInitialPose(deviation), 5);
  }
}

void PlanningInterfaceTestManager::testWithOffTrackOdometry(
  rclcpp::Node::SharedPtr target_node, const std::string & topic_name)
{
  for (const auto & deviation : {0.0, 1.0, 10.0, 100.0}) {
    publishInput(target_node, topic_name, autoware::test_utils::makeOdometry(deviation), 5);
  }
}
}  // namespace autoware::planning_test_manager
