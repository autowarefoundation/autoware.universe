/*
* Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef TEST_NONLINEAR_MPC_NODE_HPP_
#define TEST_NONLINEAR_MPC_NODE_HPP_

#include <memory>
#include <vector>
#include "nonlinear_mpc_node/nonlinear_mpc_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gtest/gtest.h"
#include "fake_test_node/fake_test_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

using VelocityMsg = nav_msgs::msg::Odometry;
using TrajectoryMsg = autoware_auto_planning_msgs::msg::Trajectory;
using ControlCmdMsg = autoware_auto_control_msgs::msg::AckermannControlCommand;
using SteeringMeasuredMsg = autoware_auto_vehicle_msgs::msg::SteeringReport;
using NonlinearMPCPerformanceMsg = autoware_auto_vehicle_msgs::msg::NonlinearMPCPerformanceReport;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using namespace std::chrono_literals;

using DelayCompensationRefs = autoware_auto_vehicle_msgs::msg::DelayCompensationRefs;
using ErrorReportMsg = autoware_auto_vehicle_msgs::msg::ControllerErrorReport;
auto constexpr EPS = std::numeric_limits<double>::epsilon();

#endif  // TEST_NONLINEAR_MPC_NODE_HPP_
