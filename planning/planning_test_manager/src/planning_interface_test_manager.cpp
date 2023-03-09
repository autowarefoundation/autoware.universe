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

// void PlanningIntefaceTestManager::testNormalBehavior(rclcpp::Node & node)
// {
//   testNominalTrajectory(node);
// }
// void PlanningIntefaceTestManager::testCheckPlaningInterface(rclcpp::Node & node)
// {
//   // publisherがいるやつだけ
//   testWithEmptyTrajectory(node);
//   testWithEmptyRoute(node);
//   testWithEmptyRoute(node);
// }

void PlanningIntefaceTestManager::setOdomTopicName(std::string topic_name)
{
  odom_pub_ = rclcpp::create_publisher<Odometry>(test_node_, topic_name, 1);
}

void PlanningIntefaceTestManager::setMaxVelocityTopicName(std::string topic_name)
{
  max_velocity_pub_ = rclcpp::create_publisher<std_msgs::msg::Float32>(test_node_, topic_name, 1);
}

void PlanningIntefaceTestManager::setPointCloudTopicName(std::string topic_name)
{
  point_cloud_pub_ = rclcpp::create_publisher<PointCloud2>(test_node_, topic_name, 1);
}

void PlanningIntefaceTestManager::setPredictedObjectsTopicName(std::string topic_name)
{
  PredictedObjects_pub_ = rclcpp::create_publisher<PredictedObjects>(test_node_, topic_name, 1);
}

void PlanningIntefaceTestManager::setTFTopicName(std::string topic_name)
{
  TF_pub_ = rclcpp::create_publisher<TFMessage>(test_node_, topic_name, 1);
}

void PlanningIntefaceTestManager::setReceivedTrajectoryTopicName(std::string topic_name)
{
  // Count the number of trajectory received.
  traj_sub_ = test_node_->create_subscription<Trajectory>(
    topic_name, 10,
    std::bind(&PlanningIntefaceTestManager::countCallback, this, std::placeholders::_1));
}

void PlanningIntefaceTestManager::setReceivedMaxVelocityTopicName(std::string topic_name)
{
  // Count the number of trajectory received.
  max_velocity_sub_ = test_node_->create_subscription<std_msgs::msg::Float32>(
    topic_name, 10,
    std::bind(&PlanningIntefaceTestManager::countCallbackMaxVelocity, this, std::placeholders::_1));
}

void PlanningIntefaceTestManager::publishAllPlanningInterfaceTopics()
{
  odom_pub_->publish(genDefaultOdom());
  point_cloud_pub_->publish(genDefaultPointCloud());
  PredictedObjects_pub_->publish(genDefaultPredictedObjects());
  TF_pub_->publish(genDefaultTFMessage());
}

void PlanningIntefaceTestManager::publishNominalTrajectory()
{
  normal_trajectory_pub_ = rclcpp::create_publisher<Trajectory>(
    test_node_, "/planning_test_manager/planning_interface_test_manager/normal_trajectory", 1);
  normal_trajectory_pub_->publish(genDefaultTrajectory());
}
void PlanningIntefaceTestManager::publishEmptyTrajectory()
{
  empty_trajectory_pub_ = rclcpp::create_publisher<Trajectory>(
    test_node_, "/planning_test_manager/planning_interface_test_manager/empty_trajectory", 1);
  empty_trajectory_pub_->publish(genDefaultTrajectory());
}

// test for normal working
void PlanningIntefaceTestManager::testNominalTrajectory(rclcpp::Node & node)
{
  publishAllPlanningInterfaceTopics();  // publish all necessary information except trajectory for
                                        // planning to work

  publishNominalTrajectory();  // publish normal trajectory
  // check that the node does not die here.
  ASSERT_NO_THROW(executeNode(node));
}

// Test node is working properly（node independent？）
// EXPECT_GE(getReceivedTrajectoryNum(), 1);

// check to see if target node is dead.
void PlanningIntefaceTestManager::testWithEmptyTrajectory(rclcpp::Node & node)
{
  publishAllPlanningInterfaceTopics();  // publish all necessary information except trajectory for
                                        // planning to work
  publishEmptyTrajectory();             // publish empty trajectory
  ASSERT_NO_THROW(executeNode(node));
}

void PlanningIntefaceTestManager::countCallback([[maybe_unused]] const Trajectory trajectory)
{
  // Increment the counter.
  ++count_;

  // Display the current count.
  RCLCPP_DEBUG(rclcpp::get_logger("PlanningInterfaceTestManager"), "Current count: %d", count_);
}

void PlanningIntefaceTestManager::countCallbackMaxVelocity(
  [[maybe_unused]] const std_msgs::msg::Float32 max_velocity)
{
  // Increment the counter.
  ++count_;

  // Display the current count.
  RCLCPP_DEBUG(rclcpp::get_logger("PlanningInterfaceTestManager"), "Current count: %d", count_);
}

int PlanningIntefaceTestManager::getReceivedTrajectoryNum() { return count_; }
int PlanningIntefaceTestManager::getReceivedMaxVelocityNum() { return count_; }
void PlanningIntefaceTestManager::executeNode(rclcpp::Node & node)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node_shared_ptr = std::shared_ptr<rclcpp::Node>(&node);
  executor.add_node(node_shared_ptr);
  executor.add_node(test_node_);
  executor.spin_some();
  rclcpp::sleep_for(std::chrono::milliseconds(100));
}
}  // namespace planning_test_manager
