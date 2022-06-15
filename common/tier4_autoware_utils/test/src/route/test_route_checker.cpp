// Copyright 2022 TIER IV, Inc.
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

#include "test_route_checker.hpp"
#include "tier4_autoware_utils/route/route_checker.hpp"


#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using tier4_autoware_utils::RouteChecker;

class CheckerNode : public rclcpp::Node
{
public:
  CheckerNode() : Node("test_checker_node")
  {
    route_checker_ = std::make_unique<RouteChecker>(this);
  }

  std::unique_ptr<RouteChecker> route_checker_;
};

class PubManager : public rclcpp::Node
{
public:
  PubManager() : Node("test_pub_node")
  {
    pub_map_ = create_publisher<HADMapBin>("/map/vector_map", 1);
    pub_route_ = create_publisher<HADMapRoute>("/planning/scenario_planning/route", 1);
  }

  rclcpp::Publisher<Odometry>::SharedPtr pub_map_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_route_;

  void publishRoute(const geometry_msgs::msg::Pose & pose, const double publish_duration)
  {
    const auto start_time = this->now();
    while (true) {
      const auto now = this->now();

      const auto time_diff = now - start_time;
      if (publish_duration < time_diff.seconds()) {
        break;
      }

      rclcpp::WallRate(10).sleep();
    }
  }


TEST(route_checker, isRouteValid)
{
  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
  

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

  
    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }
}




