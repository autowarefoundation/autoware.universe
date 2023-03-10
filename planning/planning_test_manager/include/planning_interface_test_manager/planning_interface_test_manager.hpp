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

#ifndef PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_HPP_
#define PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

namespace planning_test_manager
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using tf2_msgs::msg::TFMessage;
using tier4_planning_msgs::msg::VelocityLimit;

class PlanningIntefaceTestManager
{
public:
  PlanningIntefaceTestManager() {}

  void declareVehicleInfoParams(rclcpp::NodeOptions & node_options);
  void declareNearestSearchDistanceParams(rclcpp::NodeOptions & node_options);

  void setOdomTopicName(std::string topic_name);
  void setMaxVelocityTopicName(std::string topic_name);
  void setTrajectoryTopicName(std::string topic_name);

  void setOutputTrajectoryTopicName(std::string topic_name);
  void setOutputMaxVelocityTopicName(std::string topic_name);

  void publishOdometry(rclcpp::Node::SharedPtr node);
  void publishMaxVelocity(rclcpp::Node::SharedPtr node);

  void setTrajectorySubscriber();

  void testWithNominalTrajectory(rclcpp::Node::SharedPtr node);
  void testWithAbnormalTrajectory(
    rclcpp::Node::SharedPtr node, const Trajectory & abnormal_trajectory);

  int getReceivedTopicNum();

private:
  // Publisher
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr PredictedObjects_pub_;
  rclcpp::Publisher<TFMessage>::SharedPtr TF_pub_;
  rclcpp::Publisher<SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr normal_trajectory_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr empty_trajectory_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr max_velocity_pub_;

  // Subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<VelocityLimit>::SharedPtr max_velocity_sub_;

  std::string input_trajectory_name_;
  std::string output_trajectory_name_;

  // Node
  rclcpp::Node::SharedPtr test_node_ =
    std::make_shared<rclcpp::Node>("planning_interface_test_node");
  int count_{0};
  void countCallback(const Trajectory trajectory);
  void countCallbackMaxVelocity(const VelocityLimit max_velocity);

  Odometry genDefaultOdom() { return Odometry{}; }
  PointCloud2 genDefaultPointCloud() { return PointCloud2{}; }
  PredictedObjects genDefaultPredictedObjects() { return PredictedObjects{}; }
  TFMessage genDefaultTFMessage() { return TFMessage{}; }
  VelocityLimit genDefaultMaxVelocity() { return VelocityLimit{}; }

  Trajectory genDefaultTrajectory() { return Trajectory{}; }

  void publishNominalTrajectory(rclcpp::Node::SharedPtr node);
  void publishAbnormalTrajectory(
    rclcpp::Node::SharedPtr node, const Trajectory & abnormal_trajectory);

};  // class PlanningIntefaceTestManager

}  // namespace planning_test_manager

#endif  // PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_HPP_
