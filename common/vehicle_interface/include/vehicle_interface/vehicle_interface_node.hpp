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
/// \file
/// \brief Base class for vehicle drivers
#ifndef VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
#define VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_

#include <vehicle_interface/platform_interface.hpp>
#include <vehicle_interface/safety_state_machine.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <mpark_variant_vendor/variant.hpp>
#include <rclcpp/rclcpp.hpp>
#include <reference_tracking_controller/reference_tracking_controller.hpp>
#include <signal_filters/signal_filter.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_control_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/autonomy_mode_change.hpp>

#include <experimental/optional>
#include <chrono>
#include <exception>
#include <memory>
#include <map>
#include <string>
#include <unordered_set>

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

using Real = decltype(BasicControlCommand::long_accel_mps2);
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using autoware_auto_vehicle_msgs::msg::HeadlightsReport;
using autoware_auto_vehicle_msgs::msg::HornCommand;
using autoware_auto_vehicle_msgs::msg::HornReport;
using autoware_auto_vehicle_msgs::msg::WipersCommand;
using autoware_auto_vehicle_msgs::msg::WipersReport;

/// Convenience struct for construction
struct TopicNumMatches
{
  std::string topic;
};  // struct TopicNumMatches

/// Convenience struct to construct filters
struct FilterConfig
{
  std::string type;
  Real cutoff_frequency;
};  // struct FilterConfig

enum class ViFeature
{
  HEADLIGHTS,
  HORN,
  WIPERS,
};

/// A node which receives commands and sends them to the vehicle platform, and publishes
/// reports from the vehicle platform
class VEHICLE_INTERFACE_PUBLIC VehicleInterfaceNode : public ::rclcpp::Node
{
public:
  /// ROS 2 parameter constructor
  /// \param[in] node_name The name for the node
  /// \param[in] features Vector of features supported by this vehicle interface
  /// \param[in] options An rclcpp::NodeOptions object
  VehicleInterfaceNode(
    const std::string & node_name,
    const std::unordered_set<ViFeature> & features,
    const rclcpp::NodeOptions & options);

protected:
  using ControllerBasePtr =
    std::unique_ptr<common::reference_tracking_controller::ReferenceTrackerBase<Real>>;
  using FilterBasePtr = std::unique_ptr<common::signal_filters::FilterBase<Real>>;
  using ModeChangeRequest = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Request;
  using ModeChangeResponse = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Response;

  struct VehicleFilter
  {
    FilterBasePtr longitudinal;
    FilterBasePtr curvature;
    FilterBasePtr front_steer;
    FilterBasePtr rear_steer;
  };
  /// Set the low pass filter
  void set_filter(VehicleFilter && filter) noexcept;
  /// Set the reference tracker (controller)
  void set_reference_tracker(ControllerBasePtr && controller) noexcept;
  /// Set the vehicle-specific PlatformInterface
  void set_interface(std::unique_ptr<PlatformInterface> && interface) noexcept;
  /// Get access to logger
  rclcpp::Logger logger() const noexcept;
  /// Get access to Safety State Machine
  const SafetyStateMachine get_state_machine() const noexcept;

  /// Error handling behavior for when sending a control command has failed, default is throwing an
  /// exception, which is caught and turned into a change in the NodeState to ERROR
  /// TODO(c.ho) add command which failed to send as an argument
  virtual void on_control_send_failure();
  /// Error handling behavior for when sending a state command has failed, default is throwing an
  /// exception, which is caught and turned into a change in the NodeState to ERROR
  /// TODO(c.ho) add command which failed to send as an argument
  virtual void on_state_send_failure();
  /// Error handling behavior for when changing the autonomy mode has failed. Default is throwing
  /// an exception
  virtual void on_mode_change_failure();
  /// Error handling behavior for when receiving data from the vehicle platform has timed out,
  /// default is throwing an exception, which is caught and turned into a change in the NodeState to
  /// ERROR
  virtual void on_read_timeout();
  /// Handle exception thrown in main loop. Default behavior is to set NodeState to ERROR
  virtual void on_error(std::exception_ptr eptr);

private:
  // Helper function called in constructors
  VEHICLE_INTERFACE_LOCAL void init(
    const TopicNumMatches & control_command,
    const TopicNumMatches & state_command,
    const TopicNumMatches & odometry,
    const TopicNumMatches & state_report,
    const std::experimental::optional<StateMachineConfig> & state_machine_config,
    const FilterConfig & longitudinal_filter,
    const FilterConfig & curvature_filter,
    const FilterConfig & front_steer_filter,
    const FilterConfig & rear_steer_filter,
    const std::chrono::nanoseconds & cycle_time);

  // Run just before main loop, ensure that all invariants (possibly from child class) are enforced
  VEHICLE_INTERFACE_LOCAL void check_invariants();

  // Send state command
  VEHICLE_INTERFACE_LOCAL void send_state_command(const MaybeStateCommand & maybe_command);
  // Read data from vehicle platform for time budget, publish data
  VEHICLE_INTERFACE_LOCAL void read_and_publish();
  // Core loop for different input commands. Specialized differently for each topic type
  template<typename T>
  VEHICLE_INTERFACE_LOCAL void on_command_message(const T & msg);
  // Callback for requests to change autonomoy mode
  VEHICLE_INTERFACE_LOCAL void on_mode_change_request(
    ModeChangeRequest::SharedPtr request, ModeChangeResponse::SharedPtr response);
  /// Log a warning from the safety state machine: transition node state and/or log
  VEHICLE_INTERFACE_LOCAL void state_machine_report();

  rclcpp::TimerBase::SharedPtr m_read_timer{nullptr};
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VehicleOdometry>::SharedPtr m_odom_pub{nullptr};
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VehicleStateReport>::SharedPtr m_state_pub{
    nullptr};
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VehicleStateCommand>::SharedPtr
    m_state_sub{nullptr};
  rclcpp::Publisher<HeadlightsReport>::SharedPtr m_headlights_rpt_pub{nullptr};
  rclcpp::Subscription<HeadlightsCommand>::SharedPtr m_headlights_cmd_sub{nullptr};
  rclcpp::Publisher<HornReport>::SharedPtr m_horn_rpt_pub{nullptr};
  rclcpp::Subscription<HornCommand>::SharedPtr m_horn_cmd_sub{nullptr};
  rclcpp::Publisher<WipersReport>::SharedPtr m_wipers_rpt_pub{nullptr};
  rclcpp::Subscription<WipersCommand>::SharedPtr m_wipers_cmd_sub{nullptr};
  rclcpp::Service<autoware_auto_vehicle_msgs::srv::AutonomyModeChange>::SharedPtr m_mode_service{
    nullptr};

  using BasicSub = rclcpp::Subscription<BasicControlCommand>::SharedPtr;
  using RawSub =
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::RawControlCommand>::SharedPtr;
  using HighLevelSub =
    rclcpp::Subscription<autoware_auto_control_msgs::msg::HighLevelControlCommand>::SharedPtr;
  using AckermannSub =
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr;

  mpark::variant<RawSub, BasicSub, HighLevelSub, AckermannSub> m_command_sub{};

  std::unique_ptr<PlatformInterface> m_interface{nullptr};
  VehicleFilter m_filter{nullptr, nullptr, nullptr, nullptr};
  ControllerBasePtr m_controller{nullptr};
  std::unique_ptr<SafetyStateMachine> m_state_machine{nullptr};
  std::chrono::system_clock::time_point m_last_command_stamp{};
  std::chrono::nanoseconds m_cycle_time{};
  MaybeStateCommand m_last_state_command{};

  std::map<std::string, ViFeature> m_avail_features =
  {
    {"headlights", ViFeature::HEADLIGHTS},
    {"horn", ViFeature::HORN},
    {"wipers", ViFeature::WIPERS},
  };
  std::unordered_set<ViFeature> m_enabled_features{};
};  // class VehicleInterfaceNode

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

#endif  // VEHICLE_INTERFACE__VEHICLE_INTERFACE_NODE_HPP_
