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

#include <planning_interface_test_manager/planning_interface_test_manager.hpp>

namespace planning_test_manager
{

// PlanningIntefaceTestManager::PlanningIntefaceTestManager() {}

// PlanningIntefaceTestManager::PlanningTestManager(const std::string & odom_pub_name)
// {
//   odom_pub_ = createSubscription(nav_msgs::msg::Odometory, odom_pub_name, 1);
// }

void PlanningIntefaceTestManager::testPlaningInterface(rclcpp::Node & node)
{
  testNominalTrajectory(node);
}

void PlanningIntefaceTestManager::setOdomTopicName()
{
  odom_pub_ = rclcpp::create_publisher<Odometry>(test_node, "/kinematic_evaluator/input/twist", 1);
}

void PlanningIntefaceTestManager::setReceivedTrajectoryTopicName()
{
  traj_sub_ = test_node->create_subscription<Trajectory>(
    "count_topic", 10,
    std::bind(&PlanningIntefaceTestManager::count_callback, this, std::placeholders::_1));
}

Odometry genDefaultOdom() { return Odometry{}; }
Trajectory genDefaultTrajectory() { return Trajectory{}; }

void PlanningIntefaceTestManager::publishAllPlanningInterfaceTopics()
{
  odom_pub_->publish(genDefaultOdom());
  // ...
  // ...
}
void PlanningIntefaceTestManager::publishNominalTrajectory()
{
  normal_trajectory_pub_ = rclcpp::create_publisher<Trajectory>(
    test_node, "/planning_test_manager/planning_interface_test_manager/normal_trajectory", 1);
  normal_trajectory_pub_->publish(genDefaultTrajectory());
}
void PlanningIntefaceTestManager::publishEmptyTrajectory()
{
  empty_trajectory_pub_ = rclcpp::create_publisher<Trajectory>(
    test_node, "/planning_test_manager/planning_interface_test_manager/empty_trajectory", 1);
  empty_trajectory_pub_->publish(genDefaultTrajectory());
}

// test for normal working
void PlanningIntefaceTestManager::testNominalTrajectory(rclcpp::Node & node)
{
  publishAllPlanningInterfaceTopics();  // publish all necessary information except trajectory for
                                        // planning to work

  publishNominalTrajectory();  // publish normal trajectory
  rclcpp::executors::SingleThreadedExecutor executor;
  auto nodeSharedPtr = std::shared_ptr<rclcpp::Node>(&node);
  executor.add_node(nodeSharedPtr);
  executor.spin_once();
  ASSERT_NO_THROW(executor.spin_once());
  // trajectory is subscribed to the node when it is spin, check that the node
  // does not die here.

  // Test node is working properly（node independent？）
  EXPECT_GE(getReceivedTrajectoryNum(), 1);
}

// check to see if target node is dead.
void PlanningIntefaceTestManager::testWithEmptyTrajectory(rclcpp::Node & node)
{
  publishAllPlanningInterfaceTopics();  // publish all necessary information except trajectory for
                                        // planning to work
  publishEmptyTrajectory();             // publish empty trajectory

  rclcpp::executors::SingleThreadedExecutor executor;
  auto nodeSharedPtr = std::shared_ptr<rclcpp::Node>(&node);
  executor.add_node(nodeSharedPtr);
  executor.spin_once();
  ASSERT_NO_THROW(executor.spin_once());  // empty trajectory is subscribed to the node when it is
                                          // spin, check that the node does not die here.
}
void PlanningIntefaceTestManager::count_callback([[maybe_unused]] const Trajectory trajectory)
{
  // Increment the counter.
  ++count_;

  // Display the current count.
  RCLCPP_DEBUG(rclcpp::get_logger("PlanningInterfaceTestManager"), "Current count: %d", count_);
}
int getReceivedTrajectoryNum();

}  // namespace planning_test_manager