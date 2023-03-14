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
#include <planning_interface_test_manager/planning_interface_test_manager_utils.hpp>

namespace planning_test_manager
{
void PlanningIntefaceTestManager::declareVehicleInfoParams(rclcpp::NodeOptions & node_options)
{
  // for vehicle info
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", 3.0);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);
}

void PlanningIntefaceTestManager::declareNearestSearchDistanceParams(
  rclcpp::NodeOptions & node_options)
{
  node_options.append_parameter_override("ego_nearest_dist_threshold", 3.0);
  node_options.append_parameter_override("ego_nearest_yaw_threshold", 1.046);
}

void PlanningIntefaceTestManager::setOdomTopicName(std::string topic_name)
{
  odom_pub_ = rclcpp::create_publisher<Odometry>(test_node_, topic_name, 1);
}

void PlanningIntefaceTestManager::setMaxVelocityTopicName(std::string topic_name)
{
  max_velocity_pub_ = rclcpp::create_publisher<VelocityLimit>(test_node_, topic_name, 1);
}

void PlanningIntefaceTestManager::setTrajectoryTopicName(std::string topic_name)
{
  input_trajectory_name_ = topic_name;
}

void PlanningIntefaceTestManager::setOutputTrajectoryTopicName(std::string topic_name)
{
  output_trajectory_name_ = topic_name;
}

void PlanningIntefaceTestManager::publishOdometry(rclcpp::Node::SharedPtr target_node)
{
  odom_pub_->publish(genDefaultOdom());
  spinSomeNodes(test_node_, target_node);
}

void PlanningIntefaceTestManager::publishMaxVelocity(rclcpp::Node::SharedPtr target_node)
{
  max_velocity_pub_->publish(genDefaultMaxVelocity());
  spinSomeNodes(test_node_, target_node);
}

void PlanningIntefaceTestManager::setTrajectorySubscriber()
{
  // Count the number of trajectory received.
  traj_sub_ = test_node_->create_subscription<Trajectory>(
    output_trajectory_name_, 10,
    std::bind(
      &PlanningIntefaceTestManager::countCallback<Trajectory>, this, std::placeholders::_1));
}

// test for normal working
void PlanningIntefaceTestManager::testWithNominalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  // check that the node does not die here.
  ASSERT_NO_THROW(publishNominalTrajectory(target_node));
}

// check to see if target node is dead.
void PlanningIntefaceTestManager::testWithAbnormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const Trajectory & abnormal_trajectory)
{
  ASSERT_NO_THROW(publishAbnormalTrajectory(target_node, abnormal_trajectory));
}

int PlanningIntefaceTestManager::getReceivedTopicNum() { return count_; }

void PlanningIntefaceTestManager::publishNominalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  normal_trajectory_pub_ =
    rclcpp::create_publisher<Trajectory>(test_node_, input_trajectory_name_, 1);
  normal_trajectory_pub_->publish(test_utils::generateTrajectory<Trajectory>(10, 1.0));
  spinSomeNodes(test_node_, target_node);
  spinSomeNodes(test_node_, target_node);
}

void PlanningIntefaceTestManager::publishAbnormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const Trajectory & abnormal_trajectory)
{
  empty_trajectory_pub_ =
    rclcpp::create_publisher<Trajectory>(test_node_, input_trajectory_name_, 1);
  empty_trajectory_pub_->publish(abnormal_trajectory);
  spinSomeNodes(test_node_, target_node);
}

void PlanningIntefaceTestManager::spinSomeNodes(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node)
{
  rclcpp::spin_some(test_node);
  rclcpp::spin_some(target_node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
}

}  // namespace planning_test_manager
