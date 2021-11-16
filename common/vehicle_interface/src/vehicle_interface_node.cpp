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
#include <common/types.hpp>

#include <signal_filters/filter_factory.hpp>
#include <time_utils/time_utils.hpp>

#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>
#include <utility>

#include "vehicle_interface/vehicle_interface_node.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;


namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

////////////////////////////////////////////////////////////////////////////////
VehicleInterfaceNode::VehicleInterfaceNode(
  const std::string & node_name,
  const std::unordered_set<ViFeature> & features,
  const rclcpp::NodeOptions & options)
: Node{node_name, options}
{
  // Helper functions
  const auto topic_num_matches_from_param = [this](const auto param) {
      const auto name_param = declare_parameter<std::string>(param);
      return TopicNumMatches{name_param};
    };
  const auto time = [this](const auto param_name) -> std::chrono::milliseconds {
      const auto count_ms = declare_parameter<int64_t>(param_name);
      return std::chrono::milliseconds{count_ms};
    };
  const auto limits_from_param = [this](const auto prefix) -> Limits<float32_t> {
      const auto prefix_dot = prefix + std::string{"."};
      return {
      static_cast<float32_t>(declare_parameter<float64_t>(prefix_dot + "min")),
      static_cast<float32_t>(declare_parameter<float64_t>(prefix_dot + "max")),
      static_cast<float32_t>(declare_parameter<float64_t>(prefix_dot + "threshold"))
      };
    };
  // optionally instantiate a config
  std::experimental::optional<StateMachineConfig> state_machine_config{};
  {
    float64_t velocity_threshold;
    if (get_parameter("state_machine.gear_shift_velocity_threshold_mps", velocity_threshold)) {
      state_machine_config = StateMachineConfig{
        static_cast<float32_t>(velocity_threshold),
        limits_from_param("state_machine.acceleration_limits"),
        limits_from_param("state_machine.front_steer_limits"),
        std::chrono::milliseconds{declare_parameter<int64_t>("state_machine.time_step_ms")},
        static_cast<float32_t>(
          declare_parameter<float64_t>("state_machine.timeout_acceleration_mps2")),
        time("state_machine.state_transition_timeout_ms"),
        static_cast<float32_t>(
          declare_parameter<float64_t>("state_machine.gear_shift_accel_deadzone_mps2"))
      };
    }
  }
  // Get stuff from filter.<prefix_middle>.blah
  const auto filter = [this](const auto prefix_middle) -> FilterConfig {
      const auto prefix = std::string{"filter."} + prefix_middle + std::string{"."};
      // lazy optional stuff: if one is missing then give up
      std::string type;
      if (!get_parameter(prefix + "type", type)) {
        return FilterConfig{"", 0.0F};
      }
      const auto cutoff =
        static_cast<Real>(declare_parameter<float32_t>(prefix + "cutoff_frequency_hz"));
      return FilterConfig{type, cutoff};
    };
  // Check for enabled features
  std::vector<std::string> feature_list_string; 

  if (get_parameter("features", feature_list_string)) {
    for (const auto & feature : feature_list_string) {
      const auto found_feature = m_avail_features.find(feature);

      if (found_feature == m_avail_features.end()) {
        throw std::domain_error{"Provided feature not found in list of available features"};
      }

      const auto supported_feature = features.find(found_feature->second);

      if (supported_feature == features.end()) {
        throw std::domain_error{"Provided feature not found in list of supported features"};
      }

      m_enabled_features.insert(*supported_feature);
    }
  }

  // Actually init
  init(
    topic_num_matches_from_param("control_command"),
    TopicNumMatches{"state_command"},
    TopicNumMatches{"odometry"},
    TopicNumMatches{"state_report"},
    state_machine_config,
    filter("longitudinal"),
    filter("curvature"),
    filter("front_steer"),
    filter("rear_steer"),
    time("cycle_time_ms")
  );
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::set_filter(VehicleFilter && filter) noexcept
{
  m_filter = std::forward<VehicleFilter &&>(filter);
}

void VehicleInterfaceNode::set_reference_tracker(ControllerBasePtr && controller) noexcept
{
  m_controller = std::forward<ControllerBasePtr &&>(controller);
}

void VehicleInterfaceNode::set_interface(std::unique_ptr<PlatformInterface> && interface) noexcept
{
  m_interface = std::forward<std::unique_ptr<PlatformInterface>&&>(interface);
}

rclcpp::Logger VehicleInterfaceNode::logger() const noexcept {return get_logger();}

const SafetyStateMachine VehicleInterfaceNode::get_state_machine() const noexcept
{
  return *m_state_machine;
}

////////////////////////////////////////////////////////////////////////////////
// 9073 appears to be a false positive, or the compiler being overly pedantic for templates
// specializations
// For 1576, I argue that the damage to readability and maintainability outweighs the slim
// possibility of loading the wrong implementation:
// The former aspect is obvious (lots of code in the header); the latter half of the argument is as
// follows:
//   The generic template implementation is not defined, and it is a private member function.
//   This implies that the only way to call it is via a public interface (which is not templated).
//   Further, the only way to get a different specialization at this point is if you load the wrong
//   library or you somehow get an LD_PRELOAD. In either case, you have significantly worse problems
//   which are not special to putting the specialization in the source file.
//   Finally, because the template is only defined in a source file and compiled, there's little
//   chance for arbitrary inclusion order (without LD_PRELOAD as above; which implies other issues)
/*lint -save -e1576 See above*/
/*lint -save -e9073 see above*/
template<>
void VehicleInterfaceNode::on_command_message(
  const autoware_auto_vehicle_msgs::msg::RawControlCommand & msg)
{
  if (!m_interface->send_control_command(msg)) {
    on_control_send_failure();
  }
  send_state_command(m_last_state_command);
  m_last_state_command = MaybeStateCommand{};
}

////////////////////////////////////////////////////////////////////////////////
template<>
void VehicleInterfaceNode::on_command_message(
  const autoware_auto_control_msgs::msg::AckermannControlCommand & msg)
{
  const auto stamp = time_utils::from_message(msg.stamp);
  const auto dt = stamp - m_last_command_stamp;

  // Time should not go backwards
  if (dt < std::chrono::nanoseconds::zero()) {
    throw std::domain_error{"Vehicle interface command went backwards in time!"};
  }

  if (!m_interface->send_control_command(msg)) {
    on_control_send_failure();
  }
  send_state_command(m_last_state_command);
  m_last_state_command = MaybeStateCommand{};
}

////////////////////////////////////////////////////////////////////////////////
template<>
void VehicleInterfaceNode::on_command_message(
  const autoware_auto_vehicle_msgs::msg::VehicleControlCommand & msg)
{
  const auto stamp = time_utils::from_message(msg.stamp);
  const auto dt = stamp - m_last_command_stamp;

  // Time should not go backwards
  if (dt < std::chrono::nanoseconds::zero()) {
    throw std::domain_error{"Vehicle interface command went backwards in time!"};
  }

  // Hit command with low pass filter TODO(c.ho) Don't repeat yourself on stamp conversion
  // Continue with filter using message only if dt>0
  if (dt > std::chrono::nanoseconds::zero()) {
    const auto maybe_state_command = m_last_state_command;
    m_last_state_command = MaybeStateCommand{};
    m_last_command_stamp = stamp;
    const auto filter = [dt](auto & filter_ptr, auto & val) -> void {
        if (filter_ptr) {val = filter_ptr->filter(val, dt);}
      };
    auto cmd = msg;
    filter(m_filter.longitudinal, cmd.long_accel_mps2);
    filter(m_filter.front_steer, cmd.front_wheel_angle_rad);
    filter(m_filter.rear_steer, cmd.rear_wheel_angle_rad);
    // Hit commands with state machine
    const auto commands = m_state_machine->compute_safe_commands({cmd, maybe_state_command});
    // Send
    if (!m_interface->send_control_command(commands.control())) {
      on_control_send_failure();
    }
    send_state_command(commands.state());
  } else {
    RCLCPP_WARN(logger(), "Vehicle interface time did not increase, skipping");
  }
}

////////////////////////////////////////////////////////////////////////////////
//lint -e{1762} NOLINT see above, not implemented
template<>
void VehicleInterfaceNode::on_command_message(
  const autoware_auto_control_msgs::msg::HighLevelControlCommand & msg)
{
  (void)msg;
  throw std::logic_error{"Not yet implemented"};
}
/*lint -restore*/

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_mode_change_request(
  ModeChangeRequest::SharedPtr request,
  ModeChangeResponse::SharedPtr response)
{
  // Response is std_msgs::msg::Empty because changing the autonomy state
  // takes a non-trivial amount of time and the current state should be
  // reported via the VehicleStateReport
  (void)response;
  if (!m_interface->handle_mode_change_request(request)) {
    on_mode_change_failure();
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::init(
  const TopicNumMatches & control_command,
  const TopicNumMatches & state_command,
  const TopicNumMatches & odometry,
  const TopicNumMatches & state_report,
  const std::experimental::optional<StateMachineConfig> & state_machine_config,
  const FilterConfig & longitudinal_filter,
  const FilterConfig & curvature_filter,
  const FilterConfig & front_steer_filter,
  const FilterConfig & rear_steer_filter,
  const std::chrono::nanoseconds & cycle_time)
{
  m_cycle_time = cycle_time;
  // Timer
  m_read_timer = create_wall_timer(
    m_cycle_time, [this]() {
      try {
        read_and_publish();
      } catch (...) {
        on_error(std::current_exception());
      }
    });
  // Make publishers
  m_state_pub = create_publisher<autoware_auto_vehicle_msgs::msg::VehicleStateReport>(
    state_report.topic + "_out", rclcpp::QoS{10U});
  m_odom_pub =
    create_publisher<autoware_auto_vehicle_msgs::msg::VehicleOdometry>(odometry.topic, rclcpp::QoS{10U});
  // Make subordinate subscriber TODO(c.ho) parameterize time better
  using VSC = autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
  m_state_sub = create_subscription<VSC>(
    state_command.topic, rclcpp::QoS{10U},
    [this](VSC::SharedPtr msg) {m_last_state_command = *msg;});

  // Feature subscriptions/publishers
  if (m_enabled_features.find(ViFeature::HEADLIGHTS) != m_enabled_features.end()) {
    m_headlights_rpt_pub = create_publisher<autoware_auto_vehicle_msgs::msg::HeadlightsReport>(
      "headlights_report", rclcpp::QoS{10U});
    m_headlights_cmd_sub = create_subscription<autoware_auto_vehicle_msgs::msg::HeadlightsCommand>(
      "headlights_command", rclcpp::QoS{10U},
      [this](autoware_auto_vehicle_msgs::msg::HeadlightsCommand::SharedPtr msg)
      {m_interface->send_headlights_command(*msg);});
  }

  if (m_enabled_features.find(ViFeature::HORN) != m_enabled_features.end()) {
    m_horn_rpt_pub = create_publisher<autoware_auto_vehicle_msgs::msg::HornReport>(
      "horn_report", rclcpp::QoS{10U});
    m_horn_cmd_sub = create_subscription<autoware_auto_vehicle_msgs::msg::HornCommand>(
      "horn_command", rclcpp::QoS{10U},
      [this](autoware_auto_vehicle_msgs::msg::HornCommand::SharedPtr msg)
      {m_interface->send_horn_command(*msg);});
  }

  if (m_enabled_features.find(ViFeature::WIPERS) != m_enabled_features.end()) {
    m_wipers_rpt_pub = create_publisher<autoware_auto_vehicle_msgs::msg::WipersReport>(
      "wipers_report", rclcpp::QoS{10U});
    m_wipers_cmd_sub = create_subscription<autoware_auto_vehicle_msgs::msg::WipersCommand>(
      "wipers_command", rclcpp::QoS{10U},
      [this](autoware_auto_vehicle_msgs::msg::WipersCommand::SharedPtr msg)
      {m_interface->send_wipers_command(*msg);});
  }

  // State machine boilerplate for better errors
  const auto state_machine = [&state_machine_config]() -> auto {
      if (!state_machine_config) {
        throw std::domain_error{
                "Basic or high level control command requested, but state machine not specified"};
      }
      return std::make_unique<SafetyStateMachine>(state_machine_config.value());
    };
  const auto cmd_callback = [this](auto t) -> auto {
      using Ptr = typename decltype(t)::SharedPtr;
      return [this](Ptr msg) -> void {
               try {
                 on_command_message(*msg);
               } catch (...) {
                 on_error(std::current_exception());
               }
             };
    };
  if (control_command.topic == "high_level") {
    using HCC = autoware_auto_control_msgs::msg::HighLevelControlCommand;
    m_command_sub =
      create_subscription<HCC>("high_level_command", rclcpp::QoS{10U}, cmd_callback(HCC{}));
    m_state_machine = state_machine();
  } else if (control_command.topic == "basic") {
    RCLCPP_WARN(
      logger(),
      "Use of basic control command is deprecated in favor of AckermannControlCommand");
    m_command_sub = create_subscription<BasicControlCommand>(
      "vehicle_command", rclcpp::QoS{10U}, cmd_callback(BasicControlCommand{}));
    m_state_machine = state_machine();
  } else if (control_command.topic == "ackermann") {
    using AckermannCC = autoware_auto_control_msgs::msg::AckermannControlCommand;
    m_command_sub = create_subscription<AckermannCC>(
      "ackermann_vehicle_command", rclcpp::QoS{10U}, cmd_callback(AckermannCC{}));
    m_state_machine = state_machine();
  } else if (control_command.topic == "raw") {
    using RCC = autoware_auto_vehicle_msgs::msg::RawControlCommand;
    m_command_sub =
      create_subscription<RCC>("raw_command", rclcpp::QoS{10U}, cmd_callback(RCC{}));
  } else {
    throw std::domain_error{"Vehicle interface must have exactly one command subscription"};
  }
  // Create services
  m_mode_service = create_service<autoware_auto_vehicle_msgs::srv::AutonomyModeChange>(
    "autonomy_mode", [this](
      ModeChangeRequest::SharedPtr request,
      ModeChangeResponse::SharedPtr response) -> void
    {
      on_mode_change_request(request, response);
    });
  // Make filters
  const auto create_filter = [](const auto & config) -> auto {
      using common::signal_filters::FilterFactory;
      return FilterFactory::create<Real>(config.type, config.cutoff_frequency);
    };
  m_filter.longitudinal = create_filter(longitudinal_filter);
  if (2U == m_command_sub.index()) {  // high level command
    m_filter.curvature = create_filter(curvature_filter);
  } else if (1U == m_command_sub.index()) {  // Basic command
    m_filter.front_steer = create_filter(front_steer_filter);
    m_filter.rear_steer = create_filter(rear_steer_filter);
  } else {  // Raw command
    // Nothing
  }

  check_invariants();
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::check_invariants()
{
  if (decltype(m_cycle_time)::zero() >= m_cycle_time) {
    throw std::domain_error{"Cycle time must be positive"};
  }
  // Check command sub
  const auto ctrl_not_null =
    mpark::visit([](auto && sub) -> bool8_t {return static_cast<bool8_t>(sub);}, m_command_sub);
  if (!ctrl_not_null) {
    throw std::domain_error{"Vehicle interface must have exactly one command subscription"};
  }
  // Check interface
  // TODO(c.ho) reenable check when check_invariants is moved to some kind of "Activating" thing
  // i.e. LifecycleNode
  // if (!m_interface) {
  //   throw std::domain_error{"Vehicle interface must have a platform interface"};
  // }
  // Check state machine
  if (0U == m_command_sub.index()) {
    if (m_state_machine) {
      // Warn
      RCLCPP_WARN(
        logger(), "State machine instantiated for raw control vehicle interface, "
        "will not be used");
    }
  } else {  // basic or high level command
    if (!m_state_machine) {
      throw std::logic_error{"Vehicle interface should have instantiated a state machine!"};
    }
  }
  // Check high level controller
  if ((!m_controller) && (2U == m_command_sub.index())) {
    throw std::domain_error{"Vehicle interface must have a controller for high level control"};
  }
}


////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::send_state_command(const MaybeStateCommand & maybe_command)
{
  if (maybe_command) {
    if (!m_interface->send_state_command(maybe_command.value())) {
      on_state_send_failure();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::read_and_publish()
{
  if (!m_interface->update(m_cycle_time - std::chrono::milliseconds{2LL})) {
    on_read_timeout();
  }
  // Publish data from interface
  m_odom_pub->publish(m_interface->get_odometry());
  m_state_pub->publish(m_interface->get_state_report());

  // Publish feature reports
  if (m_headlights_rpt_pub) {
    m_headlights_rpt_pub->publish(m_interface->get_headlights_report());
  }

  if (m_wipers_rpt_pub) {
    m_wipers_rpt_pub->publish(m_interface->get_wipers_report());
  }

  // Update
  if (m_state_machine) {
    m_state_machine->update(m_interface->get_odometry(), m_interface->get_state_report());
    state_machine_report();
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_control_send_failure()
{
  throw std::runtime_error{"Sending control command failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_state_send_failure()
{
  throw std::runtime_error{"Sending state command failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_read_timeout()
{
  throw std::runtime_error{"Receiving data failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_mode_change_failure()
{
  throw std::runtime_error{"Changing autonomy mode failed"};
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::on_error(std::exception_ptr eptr)
{
  try {
    std::rethrow_exception(eptr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), e.what());
  } catch (...) {
    RCLCPP_ERROR(logger(), "VehicleInterface: Unknown error!");
  }
}

////////////////////////////////////////////////////////////////////////////////
void VehicleInterfaceNode::state_machine_report()
{
  for (const auto report : m_state_machine->reports()) {
    switch (report) {
      case StateMachineReport::CLAMP_PAST_THRESHOLD:
        RCLCPP_WARN(logger(), "Control command wildly out of range");
        break;
      case StateMachineReport::BAD_STATE:
        RCLCPP_WARN(logger(), "Bad state command sanitized");
        break;
      case StateMachineReport::WIPERS_ON_HEADLIGHTS_ON:
        RCLCPP_INFO(logger(), "Added headlights on due to wipers on");
        break;
      case StateMachineReport::REMOVE_GEAR_COMMAND:
        RCLCPP_WARN(logger(), "Bad gear command removed");
        break;
      case StateMachineReport::HIGH_FREQUENCY_ACCELERATION_COMMAND:
        {
          const auto err_str = "High frequency acceleration command";
          RCLCPP_ERROR(logger(), err_str);
        }
        break;
      case StateMachineReport::HIGH_FREQUENCY_STEER_COMMAND:
        {
          const auto err_str = "High frequency steering command";
          RCLCPP_ERROR(logger(), err_str);
        }
        break;
      case StateMachineReport::HIGH_FREQUENCY_VELOCITY_REPORT:
        {
          const auto err_str = "High frequency velocity report";
          RCLCPP_ERROR(logger(), err_str);
        }
        RCLCPP_WARN(logger(), "Control command wildly out of range");
        break;
      case StateMachineReport::HIGH_FREQUENCY_STEER_REPORT:
        {
          const auto err_str = "High frequency steering report";
          RCLCPP_ERROR(logger(), err_str);
        }
        break;
      case StateMachineReport::STATE_TRANSITION_TIMEOUT:
        // TODO(JWhitleyWork) Re-enable this error when we can
        // get it under control.
        // RCLCPP_ERROR(logger(), "State transition timed out");
        break;
      default:
        throw std::logic_error{"Bad state machine report"};
    }
  }
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware
