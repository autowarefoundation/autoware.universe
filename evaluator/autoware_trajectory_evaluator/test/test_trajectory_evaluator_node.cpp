// Copyright 2025 TIER IV, Inc.
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sstream>
#include <nav_msgs/msg/odometry.hpp>
#include "autoware/trajectory_evaluator/trajectory_evaluator_node.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "tier4_metric_msgs/msg/metric_array.hpp"  

using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using MetricMsg = tier4_metric_msgs::msg::Metric;

class TrajectoryEvaluatorTests : public ::testing::Test {
protected:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Subscription<MetricArrayMsg>::SharedPtr metric_subscription;
    MetricArrayMsg::SharedPtr received_metrics;
    std::shared_ptr<trajectory_evaluator::TrajectoryEvaluatorNode> evaluator_node;

    virtual void SetUp() {
        rclcpp::init(0, nullptr);
        node = rclcpp::Node::make_shared("trajectory_evaluator_test_node");

        trajectory_publisher = node->create_publisher<autoware_planning_msgs::msg::Trajectory>("/input_trajectory", 10);
        odometry_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
        metric_subscription = node->create_subscription<MetricArrayMsg>(
            "/trajectory_metrics", 10,
            [this](MetricArrayMsg::SharedPtr msg) { this->received_metrics = msg; });

        evaluator_node = std::make_shared<trajectory_evaluator::TrajectoryEvaluatorNode>(rclcpp::NodeOptions());
    }

    virtual void TearDown() {
        rclcpp::shutdown();
    }

    void publishTrajectoryAndWait(const autoware_planning_msgs::msg::Trajectory &trajectory, std::chrono::milliseconds timeout) {
        // received_metrics.reset();
        trajectory_publisher->publish(trajectory);

        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < timeout) {
            rclcpp::spin_some(node);
            rclcpp::spin_some(evaluator_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void simulateEgoVehicle(
        const autoware_planning_msgs::msg::Trajectory &trajectory, 
        double interval_seconds) 
    {
        nav_msgs::msg::Odometry odometry_msg;
        odometry_msg.header.frame_id = "map";
        odometry_msg.child_frame_id = "base_link";

        rclcpp::Time current_time = node->now(); 

        for (size_t i = 1; i < trajectory.points.size(); ++i) {
            const auto &previous_point = trajectory.points[i - 1];
            const auto &current_point = trajectory.points[i];

            double dx = current_point.pose.position.x - previous_point.pose.position.x;
            double dy = current_point.pose.position.y - previous_point.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            double velocity = current_point.longitudinal_velocity_mps;

            double travel_time = distance / velocity;

            int num_steps = static_cast<int>(std::ceil(travel_time / interval_seconds));
            for (int step = 0; step <= num_steps; ++step) {
                double t = static_cast<double>(step) / num_steps; 

                odometry_msg.pose.pose.position.x = previous_point.pose.position.x + t * dx;
                odometry_msg.pose.pose.position.y = previous_point.pose.position.y + t * dy;
                odometry_msg.pose.pose.position.z = previous_point.pose.position.z;

                odometry_msg.twist.twist.linear.x = velocity;

                current_time += rclcpp::Duration::from_seconds(interval_seconds);
                odometry_msg.header.stamp = current_time;

                odometry_publisher->publish(odometry_msg);

                rclcpp::spin_some(node);
                rclcpp::spin_some(evaluator_node);
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<int>(interval_seconds * 1000)));
            }
        }
    }

    double extractMetricValue(const MetricArrayMsg &metrics, const std::string &name) {
        for (const auto &metric : metrics.metric_array) {
            if (metric.name == name) {
                return std::stod(metric.value);
            }
        }
        throw std::runtime_error("Metric not found: " + name);
    }

    autoware_planning_msgs::msg::Trajectory generateLinearTrajectory(double length, int num_points, double velocity) {
        autoware_planning_msgs::msg::Trajectory trajectory;
        for (int i = 0; i < num_points; ++i) {
            autoware_planning_msgs::msg::TrajectoryPoint point;
            point.pose.position.x = i * (length / (num_points - 1));
            point.pose.position.y = 0.0;
            point.pose.position.z = 0.0;
            point.longitudinal_velocity_mps = velocity;
            trajectory.points.push_back(point);
        }
        return trajectory;
    }
};

TEST_F(TrajectoryEvaluatorTests, TestLinearTrajectoryWithEgoVehicleOdometry) {
    double length = 10.0;
    int num_points = 20;
    double velocity = 5.0;

    auto trajectory = generateLinearTrajectory(length, num_points, velocity);
    publishTrajectoryAndWait(trajectory, std::chrono::milliseconds(5000));

    double interval_seconds = 0.1;  
    simulateEgoVehicle(trajectory, interval_seconds);

    double expected_time = extractMetricValue(*received_metrics, "trajectory_metrics/expected_time");
    double actual_time = extractMetricValue(*received_metrics, "trajectory_metrics/actual_time");
    double time_error = extractMetricValue(*received_metrics, "trajectory_metrics/time_error");

    double tolerance = 0.1;
    EXPECT_NEAR(actual_time, expected_time, tolerance);
    EXPECT_GE(time_error, 0.0);
}
