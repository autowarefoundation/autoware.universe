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

#ifndef PLANNING_TEST_MANAGER_HPP_
#define PLANNING_TEST_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace planning_test_manager
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
// using autoware_auto_perception_msgs::msg::PredictedObjects;
// using autoware_auto_planning_msgs::msg::Trajectory;
// using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class PlanningIntefaceTestManager
{
public:
  PlanningIntefaceTestManager() {}

  void testPlaningInterface(rclcpp::Node & node);

  void setOdomTopicName(std::string topic_name);
  void setReceivedTrajectoryTopicName(std::string topic_name);

private:
  // Publisher
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr normal_trajectory_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr empty_trajectory_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;

  // Subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr
    traj_sub_;  


  // Node
  rclcpp::Node::SharedPtr test_node =
    std::make_shared<rclcpp::Node>("planning_interface_test_node");
  int count_{0};
  void count_callback(const Trajectory trajectory);

  Odometry genDefaultOdom() { return Odometry{}; }
  Trajectory genDefaultTrajectory() { return Trajectory{}; }

  void publishAllPlanningInterfaceTopics();

  void publishNominalTrajectory();
  void publishEmptyTrajectory();

  void testNominalTrajectory(rclcpp::Node & node);
  void testWithEmptyTrajectory(rclcpp::Node & node);

  int getReceivedTrajectoryNum();
};  // class PlanningIntefaceTestManager

}  // namespace planning_test_manager

#endif  // PLANNING_TEST_MANAGER_HPP_
