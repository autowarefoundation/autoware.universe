// Copyright 2023 The Autoware Foundation
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

#ifndef PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_
#define PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>

#include <boost/optional.hpp>

#include <tf2_ros/buffer.h>

#include <limits>
#include <memory>
#include <string>

namespace test_utils
{
using planning_interface::Route;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_planning_msgs::msg::Scenario;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::OccupancyGrid;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

template <class T>
T generateTrajectory(
  const size_t num_points, const double point_interval, const double velocity = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0,
  const size_t overlapping_point_index = std::numeric_limits<size_t>::max())
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = velocity;
    traj.points.push_back(p);

    if (i == overlapping_point_index) {
      Point value_to_insert = traj.points[overlapping_point_index];
      traj.points.insert(traj.points.begin() + overlapping_point_index + 1, value_to_insert);
    }
  }

  return traj;
}

void spinSomeNodes(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node,
  const int repeat_count = 1)
{
  for (int i = 0; i < repeat_count; i++) {
    rclcpp::spin_some(test_node);
    rclcpp::spin_some(target_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

template <typename T>
void setPublisher(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<T>> & publisher)
{
  publisher = rclcpp::create_publisher<T>(test_node, topic_name, 1);
}

template <typename T>
void publishData(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<T>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  publisher->publish(T{});
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<Scenario>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<Scenario>::SharedPtr publisher)
{
  auto current_scenario = std::make_shared<Scenario>();
  current_scenario->activating_scenarios.emplace_back(Scenario::PARKING);
  setPublisher(test_node, topic_name, publisher);
  publisher->publish(*current_scenario);
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<Odometry>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<Odometry>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  std::shared_ptr<Odometry> current_odometry = std::make_shared<Odometry>();
  current_odometry->header.frame_id = "base_link";
  publisher->publish(*current_odometry);
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<OccupancyGrid>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<OccupancyGrid>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  std::shared_ptr<OccupancyGrid> current_occupancy_grid = std::make_shared<OccupancyGrid>();
  current_occupancy_grid->header.frame_id = "base_link";
  publisher->publish(*current_occupancy_grid);
  spinSomeNodes(test_node, target_node);
}

template <typename T>
void setSubscriber(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Subscription<T>> & subscriber, size_t & count)
{
  // Count the number of topic received.
  subscriber =
    test_node->create_subscription<T>(topic_name, 10, [&count](const typename T::SharedPtr) {
      count++;
      std::cerr << "count up XXXX " << __FILE__ << __LINE__ << std::endl;
    });
}

}  // namespace test_utils

#endif  // PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_
