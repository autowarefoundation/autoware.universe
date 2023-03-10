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
  return;
}

void PlanningIntefaceTestManager::declareNearestSearchDistanceParams(
  rclcpp::NodeOptions & node_options)
{
  node_options.append_parameter_override("ego_nearest_dist_threshold", 3.0);
  node_options.append_parameter_override("ego_nearest_yaw_threshold", 1.046);
  return;
}

void PlanningIntefaceTestManager::setOdomTopicName(std::string topic_name)
{
  odom_pub_ = rclcpp::create_publisher<Odometry>(test_node_, topic_name, 1);
  return;
}

void PlanningIntefaceTestManager::setMaxVelocityTopicName(std::string topic_name)
{
  max_velocity_pub_ = rclcpp::create_publisher<VelocityLimit>(test_node_, topic_name, 1);
  return;
}

void PlanningIntefaceTestManager::setPointCloudTopicName(std::string topic_name)
{
  point_cloud_pub_ = rclcpp::create_publisher<PointCloud2>(test_node_, topic_name, 1);
  return;
}

void PlanningIntefaceTestManager::setPredictedObjectsTopicName(std::string topic_name)
{
  PredictedObjects_pub_ = rclcpp::create_publisher<PredictedObjects>(test_node_, topic_name, 1);
  return;
}

void PlanningIntefaceTestManager::setTFTopicName(std::string topic_name)
{
  TF_pub_ = rclcpp::create_publisher<TFMessage>(test_node_, topic_name, 1);
  return;
}

void PlanningIntefaceTestManager::setTrajectoryTopicName(std::string topic_name)
{
  input_trajectory_name_ = topic_name;
  return;
}

void PlanningIntefaceTestManager::setOutputTrajectoryTopicName(std::string topic_name)
{
  output_trajectory_name_ = topic_name;
  return;
}

void PlanningIntefaceTestManager::setOutputMaxVelocityTopicName(std::string topic_name)
{
  // Count the number of max_velocity received.
  max_velocity_sub_ = test_node_->create_subscription<VelocityLimit>(
    topic_name, 10,
    std::bind(&PlanningIntefaceTestManager::countCallbackMaxVelocity, this, std::placeholders::_1));
}

void PlanningIntefaceTestManager::publishOdometry(rclcpp::Node::SharedPtr node)
{
  odom_pub_->publish(genDefaultOdom());
  rclcpp::spin_some(test_node_);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  return;
}

void PlanningIntefaceTestManager::publishMaxVelocity(rclcpp::Node::SharedPtr node)
{
  max_velocity_pub_->publish(genDefaultMaxVelocity());
  rclcpp::spin_some(test_node_);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  return;
}



void PlanningIntefaceTestManager::setTrajectorySubscriber()
{
  // Count the number of trajectory received.
  traj_sub_ = test_node_->create_subscription<Trajectory>(
    output_trajectory_name_, 10,
    std::bind(&PlanningIntefaceTestManager::countCallback, this, std::placeholders::_1));
  return;
}

// test for normal working
void PlanningIntefaceTestManager::testWithNominalTrajectory(rclcpp::Node::SharedPtr node)
{
  // check that the node does not die here.
  ASSERT_NO_THROW(publishNominalTrajectory(node));
  return;
}

// check to see if target node is dead.
void PlanningIntefaceTestManager::testWithAbnormalTrajectory(
  rclcpp::Node::SharedPtr node, const Trajectory & abnormal_trajectory)
{
  ASSERT_NO_THROW(publishAbnormalTrajectory(node, abnormal_trajectory));
  return;
}

void PlanningIntefaceTestManager::publishNominalTrajectory(rclcpp::Node::SharedPtr node)
{
  normal_trajectory_pub_ =
    rclcpp::create_publisher<Trajectory>(test_node_, input_trajectory_name_, 1);
  normal_trajectory_pub_->publish(test_utils::generateTrajectory<Trajectory>(10, 1.0));
  rclcpp::spin_some(test_node_);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(test_node_);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  return;
}

void PlanningIntefaceTestManager::publishAbnormalTrajectory(
  rclcpp::Node::SharedPtr node, const Trajectory & abnormal_trajectory)
{
  empty_trajectory_pub_ =
    rclcpp::create_publisher<Trajectory>(test_node_, input_trajectory_name_, 1);
  empty_trajectory_pub_->publish(abnormal_trajectory);
  rclcpp::spin_some(test_node_);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  return;
}

void PlanningIntefaceTestManager::countCallback([[maybe_unused]] const Trajectory trajectory)
{
  // Increment the counter.
  ++count_;

  // Display the current count.
  RCLCPP_ERROR(rclcpp::get_logger("PlanningInterfaceTestManager"), "Current count: %d", count_);
}

void PlanningIntefaceTestManager::countCallbackMaxVelocity(
  [[maybe_unused]] const VelocityLimit max_velocity)
{
  // Increment the counter.
  ++count_;

  // Display the current count.
  RCLCPP_DEBUG(rclcpp::get_logger("PlanningInterfaceTestManager"), "Current count: %d", count_);
}

int PlanningIntefaceTestManager::getReceivedTopicNum() { return count_; }

}  // namespace planning_test_manager
