// Copyright 2021 TierIV
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

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include "stop_filter/stop_filter.hpp"
#include "rclcpp/logging.hpp"

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on
using std::placeholders::_1;

StopFilter::StopFilter(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  vx_threshold_ = declare_parameter("vx_threshold", 0.01);
  wz_threshold_ = declare_parameter("wz_threshold", 0.01);

  sub_twist_ =
    create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist",
    1,
    std::bind(&StopFilter::callbackTwistStamped, this, _1));
  sub_twist_with_covariance_ =
    create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist_with_covariance",
    1,
    std::bind(&StopFilter::callbackTwistWithCovarianceStamped, this, _1));

  pub_twist_ =
    create_publisher<geometry_msgs::msg::TwistStamped>("output/twist", 1);
  pub_twist_with_covariance_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "output/twist_with_covariance", 1);
  pub_stop_flag_ =
    create_publisher<autoware_debug_msgs::msg::BoolStamped>("debug/stop_flag", 1);
}


void StopFilter::callbackTwistStamped(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  autoware_debug_msgs::msg::BoolStamped stop_flag_msg;
  stop_flag_msg.stamp = msg->header.stamp;
  stop_flag_msg.data = false;

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header = msg->header;
  twist_msg.twist = msg->twist;

  if (
    std::fabs(msg->twist.linear.x) < vx_threshold_ &&
    std::fabs(msg->twist.angular.z) < wz_threshold_)
  {
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.angular.z = 0.0;
    stop_flag_msg.data = true;
  }
  pub_stop_flag_->publish(stop_flag_msg);
  pub_twist_->publish(twist_msg);
}


void StopFilter::callbackTwistWithCovarianceStamped(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  autoware_debug_msgs::msg::BoolStamped stop_flag_msg;
  stop_flag_msg.stamp = msg->header.stamp;
  stop_flag_msg.data = false;

  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header = msg->header;
  twist_msg.twist = msg->twist;

  if (
    std::fabs(msg->twist.twist.linear.x) < vx_threshold_ &&
    std::fabs(msg->twist.twist.angular.z) < wz_threshold_)
  {
    twist_msg.twist.twist.linear.x = 0.0;
    twist_msg.twist.twist.angular.z = 0.0;
    stop_flag_msg.data = true;
  }
  pub_stop_flag_->publish(stop_flag_msg);
  pub_twist_with_covariance_->publish(twist_msg);
}
