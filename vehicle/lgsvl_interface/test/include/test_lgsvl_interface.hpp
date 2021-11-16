// Copyright 2020 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TEST_LGSVL_INTERFACE_HPP_
#define TEST_LGSVL_INTERFACE_HPP_

#include <gtest/gtest.h>
#include <lgsvl_interface/visibility_control.hpp>
#include <lgsvl_interface/lgsvl_interface.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <memory>
#include <chrono>


using Table1D = ::autoware::common::helper_functions::LookupTable1D<double>;
using VSC = autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
using VSR = autoware_auto_vehicle_msgs::msg::VehicleStateReport;

const auto sim_ctrl_cmd_topic = "test_lgsvl/vehicle_control_cmd";
const auto sim_state_cmd_topic = "test_lgsvl/vehicle_state_cmd";
const auto sim_state_rpt_topic = "test_lgsvl/state_report";
const auto sim_nav_odom_topic = "test_lgsvl/gnss_odom";
const auto sim_veh_odom_topic = "test_lgsvl/vehicle_odom";
const auto kinematic_state_topic = "test_vehicle_kinematic_state";
const auto sim_odom_child_frame = "base_link";

class LgsvlInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("lgsvl_interface_test_node", "/gtest");

    lgsvl_interface_ = std::make_unique<lgsvl_interface::LgsvlInterface>(
      *node_,
      sim_ctrl_cmd_topic,
      sim_state_cmd_topic,
      sim_state_rpt_topic,
      sim_nav_odom_topic,
      sim_veh_odom_topic,
      kinematic_state_topic,
      sim_odom_child_frame,
      Table1D({0.0, 3.0}, {0.0, 100.0}),
      Table1D({-3.0, 0.0}, {100.0, 0.0}),
      Table1D({-0.331, 0.331}, {-100.0, 100.0}));

    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

    using namespace std::chrono_literals; // NOLINT
    timer_ = node_->create_wall_timer(
      100ms,
      [this]() {
        geometry_msgs::msg::TransformStamped nav_base_tf{};
        nav_base_tf.header.stamp = node_->now();
        nav_base_tf.header.frame_id = "base_link";
        nav_base_tf.child_frame_id = "nav_base";
        broadcaster_->sendTransform(nav_base_tf);
      });
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }

public:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::unique_ptr<lgsvl_interface::LgsvlInterface> lgsvl_interface_;
  rclcpp::TimerBase::SharedPtr timer_{};
};

template<typename T>
void wait_for_subscriber(
  const T & pub_ptr,
  const uint32_t num_expected_subs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (pub_ptr->get_subscription_count() < num_expected_subs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for subscriber");
    }
  }
}


template<typename T>
void wait_for_publisher(
  const T & sub_ptr,
  const uint32_t num_expected_pubs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (sub_ptr->get_publisher_count() < num_expected_pubs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for publisher");
    }
  }
}
#endif  // TEST_LGSVL_INTERFACE_HPP_
