// Copyright 2020-2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#ifndef TEST_VI_NODE_HPP_
#define TEST_VI_NODE_HPP_

#include <common/types.hpp>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "autoware_auto_msgs/msg/headlights_command.hpp"
#include "autoware_auto_msgs/msg/wipers_command.hpp"

#include "vehicle_interface/vehicle_interface_node.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

using autoware::drivers::vehicle_interface::Limits;
using autoware::drivers::vehicle_interface::VehicleInterfaceNode;
using autoware::drivers::vehicle_interface::PlatformInterface;
using autoware::drivers::vehicle_interface::FilterConfig;
using autoware::drivers::vehicle_interface::TopicNumMatches;
using autoware::drivers::vehicle_interface::ViFeature;

using autoware_auto_msgs::msg::HeadlightsCommand;
using autoware_auto_msgs::msg::WipersCommand;
using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::AckermannControlCommand;
using ModeChangeRequest = autoware_auto_msgs::srv::AutonomyModeChange_Request;
using ModeChangeResponse = autoware_auto_msgs::srv::AutonomyModeChange_Response;

/// Fake instantiation of interface, only checks that certain things were ever called
/// Each of the overloaded functions fails in a rotating manner. After 5 iterations, the
/// sequence goes: no failures, send raw fails, send basic fails, send state fails, and update fails
class FakeInterface : public PlatformInterface
{
public:
  explicit FakeInterface(bool8_t fail)
  : PlatformInterface{},
    m_fail{fail} {}

  bool8_t update(std::chrono::nanoseconds timeout) override
  {
    (void)timeout;
    m_update_called = true;
    return (m_count % 5) != 4;
  }
  bool8_t send_state_command(const autoware_auto_msgs::msg::VehicleStateCommand & msg) override
  {
    (void)msg;
    m_state_called = true;
    m_state = msg;
    return (m_count % 5) != 3;
  }
  bool8_t send_control_command(const autoware_auto_msgs::msg::VehicleControlCommand & msg) override
  {
    if (m_fail) {
      ++m_count;
    }
    m_basic_called = true;
    m_control = msg;
    m_controls.push_back(msg);
    return (m_count % 5) != 2;
  }
  bool8_t send_control_command(const AckermannControlCommand & msg) override
  {
    if (m_fail) {
      ++m_count;
    }
    m_ackermann_called = true;
    m_ackermann_control = msg;
    m_ackermann_controls.push_back(msg);
    return (m_count % 5) != 2;
  }
  bool8_t send_control_command(const autoware_auto_msgs::msg::RawControlCommand & msg) override
  {
    if (m_fail) {
      ++m_count;
    }
    // Slightly sketchy because of threading, but I claim it's ok since I stop the threading
    // before checking this
    m_msg = msg;
    m_raw_called = true;
    return (m_count % 5) != 1;
  }
  bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request) override
  {
    // TODO(JWhitleyWork) Actually do some testing on this
    m_mode_request = *request;
    m_mode_change_called = true;
    return true;
  }

  const RawControlCommand & msg() const noexcept {return m_msg;}
  const VehicleControlCommand & control() const noexcept {return m_control;}
  const AckermannControlCommand & ackermanm_control() const noexcept {return m_ackermann_control;}
  const VehicleStateCommand & state() const noexcept {return m_state;}
  const ModeChangeRequest & mode_request() const noexcept {return m_mode_request;}
  const std::vector<VehicleControlCommand> & controls() const noexcept {return m_controls;}

  bool8_t update_called() const noexcept {return m_update_called;}
  bool8_t state_called() const noexcept {return m_state_called;}
  bool8_t basic_called() const noexcept {return m_basic_called;}
  bool8_t ackermann_called() const noexcept {return m_ackermann_called;}
  bool8_t raw_called() const noexcept {return m_raw_called;}
  bool8_t mode_change_called() const noexcept {return m_mode_change_called;}
  int32_t count() const noexcept {return m_count;}

private:
  std::atomic<bool8_t> m_update_called{false};
  std::atomic<bool8_t> m_state_called{false};
  std::atomic<bool8_t> m_basic_called{false};
  std::atomic<bool8_t> m_ackermann_called{false};
  std::atomic<bool8_t> m_raw_called{false};
  std::atomic<bool8_t> m_mode_change_called{false};
  RawControlCommand m_msg{};
  VehicleControlCommand m_control{};
  AckermannControlCommand m_ackermann_control{};
  VehicleStateCommand m_state{};
  ModeChangeRequest m_mode_request{};
  std::vector<VehicleControlCommand> m_controls{};
  std::vector<AckermannControlCommand> m_ackermann_controls{};
  int32_t m_count{};
  bool8_t m_fail;
};

/// Simple instantiation of vehicle interface using FakeInterface
class TestVINode : public VehicleInterfaceNode
{
public:
  TestVINode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    bool8_t fail = false)
  : VehicleInterfaceNode{
      node_name,
    {
      ViFeature::HEADLIGHTS,
      ViFeature::WIPERS,
    },
      rclcpp::NodeOptions(options)
      .append_parameter_override("cycle_time_ms", static_cast<int64_t>(30LL))
      .append_parameter_override("state_machine.gear_shift_velocity_threshold_mps", 0.5F)
      .append_parameter_override("state_machine.acceleration_limits.min", -3.0F)
      .append_parameter_override("state_machine.acceleration_limits.max", 3.0F)
      .append_parameter_override("state_machine.acceleration_limits.threshold", 1.0F)
      .append_parameter_override("state_machine.front_steer_limits.min", -0.331F)
      .append_parameter_override("state_machine.front_steer_limits.max", 0.331F)
      .append_parameter_override("state_machine.front_steer_limits.threshold", 0.3F)
      .append_parameter_override("state_machine.time_step_ms", static_cast<int64_t>(100LL))
      .append_parameter_override("state_machine.timeout_acceleration_mps2", 3.0F)
      .append_parameter_override("state_machine.state_transition_timeout_ms",
        static_cast<int64_t>(3000LL))
      .append_parameter_override("state_machine.gear_shift_accel_deadzone_mps2", 0.5F)
      .append_parameter_override("features", std::vector<std::string> {"headlights", "wipers"})
  }
  {
    // sketchy, but this is because the PlatformInterface generally shouldn't be exposed
    auto interface = std::make_unique<FakeInterface>(fail);
    m_interface = interface.get();
    set_interface(std::move(interface));
  }

  const FakeInterface & interface() const noexcept {return *m_interface;}
  bool8_t error_handler_called() const noexcept {return m_error_handler_called;}
  bool8_t control_handler_called() const noexcept {return m_control_send_error_handler_called;}
  bool8_t state_handler_called() const noexcept {return m_state_send_error_handler_called;}
  bool8_t timeout_handler_called() const noexcept {return m_read_timeout_handler_called;}

protected:
  class ControlError : public std::logic_error
  {
public:
    ControlError()
    : logic_error{"control"} {}
  };
  class ReadError : public std::logic_error
  {
public:
    ReadError()
    : logic_error{"read"} {}
  };
  class StateError : public std::logic_error
  {
public:
    StateError()
    : logic_error{"state"} {}
  };
  void on_control_send_failure() override
  {
    static bool8_t flag{false};
    flag = !flag;
    if (flag) {
      throw ControlError{};
    } else {
      VehicleInterfaceNode::on_control_send_failure();
    }
  }
  void on_state_send_failure() override
  {
    static bool8_t flag{false};
    flag = !flag;
    if (flag) {
      throw StateError{};
    } else {
      VehicleInterfaceNode::on_state_send_failure();
    }
  }
  void on_read_timeout() override
  {
    static bool8_t flag{false};
    flag = !flag;
    if (flag) {
      throw ReadError{};
    } else {
      VehicleInterfaceNode::on_read_timeout();
    }
  }
  void on_error(std::exception_ptr eptr) override
  {
    m_error_handler_called = true;
    try {
      std::rethrow_exception(eptr);
    } catch (const std::runtime_error &) {
      // Generic; do nothing
    } catch (const ReadError &) {
      m_read_timeout_handler_called = true;
    } catch (const ControlError &) {
      m_control_send_error_handler_called = true;
    } catch (const StateError &) {
      m_state_send_error_handler_called = true;
    }
  }

private:
  const FakeInterface * m_interface;
  bool8_t m_error_handler_called{false};
  bool8_t m_control_send_error_handler_called{false};
  bool8_t m_state_send_error_handler_called{false};
  bool8_t m_read_timeout_handler_called{false};
};  // class TestVINode

class SanityChecks : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
};

#endif  // TEST_VI_NODE_HPP_
