//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "control_command_gate.hpp"

#include "command/builtin.hpp"
#include "command/filter.hpp"
#include "command/publisher.hpp"
#include "command/source.hpp"
#include "command/subscription.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_command_gate
{

VehicleCmdFilterParam declare_filter_params(rclcpp::Node & node, const std::string & ns)
{
  VehicleCmdFilterParam p;
  p.vel_lim = node.declare_parameter<double>(ns + "vel_lim");
  p.reference_speed_points = node.declare_parameter<LimitArray>(ns + "reference_speed_points");
  p.steer_lim = node.declare_parameter<LimitArray>(ns + "steer_lim");
  p.steer_rate_lim = node.declare_parameter<LimitArray>(ns + "steer_rate_lim");
  p.lon_acc_lim = node.declare_parameter<LimitArray>(ns + "lon_acc_lim");
  p.lon_jerk_lim = node.declare_parameter<LimitArray>(ns + "lon_jerk_lim");
  p.lat_acc_lim = node.declare_parameter<LimitArray>(ns + "lat_acc_lim");
  p.lat_jerk_lim = node.declare_parameter<LimitArray>(ns + "lat_jerk_lim");
  p.actual_steer_diff_lim = node.declare_parameter<LimitArray>(ns + "actual_steer_diff_lim");
  return p;
}

ControlCmdGate::ControlCmdGate(const rclcpp::NodeOptions & options)
: Node("control_command_gate", options), diag_(this, 0.5)
{
  // Create ROS interface.
  pub_status_ =
    create_publisher<CommandSourceStatus>("~/source/status", rclcpp::QoS(1).transient_local());
  srv_select_ = create_service<SelectCommandSource>(
    "~/source/select",
    std::bind(
      &ControlCmdGate::on_select_source, this, std::placeholders::_1, std::placeholders::_2));

  const auto on_change_source = [this](const std::string & source) {
    CommandSourceStatus status;
    status.stamp = now();
    status.source = source;
    pub_status_->publish(status);
  };

  selector_ = std::make_unique<CommandSelector>(get_logger(), on_change_source);
  diag_.setHardwareID("none");

  TimeoutDiag::Params params;
  params.warn_duration_ = declare_parameter<double>("diag_timeout_warn_duration");
  params.error_duration_ = declare_parameter<double>("diag_timeout_error_duration");

  auto nominal_filter_params = declare_filter_params(*this, "nominal_filter.");
  auto transition_filter_params = declare_filter_params(*this, "transition_filter.");
  {
    const auto info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    nominal_filter_params.wheel_base = info.wheel_base_m;
    transition_filter_params.wheel_base = info.wheel_base_m;
  }

  const auto inputs = declare_parameter<std::vector<std::string>>("inputs");
  if (std::find(inputs.begin(), inputs.end(), builtin) != inputs.end()) {
    throw std::invalid_argument("input name '" + builtin + "' is reserved");
  }

  // Create shared data buffer.
  const auto prev_control = std::make_shared<Control>();

  // Create command sources.
  {
    std::vector<std::unique_ptr<CommandSource>> sources;
    {
      auto source = std::make_unique<BuiltinEmergency>(builtin, *this);
      source->set_prev_control(prev_control);
      sources.push_back(std::move(source));
    }
    for (const auto & input : inputs) {
      auto source = std::make_unique<CommandSubscription>(input, *this);
      sources.push_back(std::move(source));
    }
    for (auto & source : sources) {
      diag_.add(*source->create_diag_task(params, *get_clock()));
      selector_->add_source(std::move(source));
    }
  }

  // Create command output.
  {
    auto output = std::make_unique<CommandPublisher>(*this);
    output->set_prev_control(prev_control);
    diag_.add(*output->create_diag_task(params, *get_clock()));

    auto filter = std::make_unique<CommandFilter>(std::move(output), *this);
    filter->set_nominal_filter_params(nominal_filter_params);
    filter->set_transition_filter_params(transition_filter_params);
    selector_->set_output(std::move(filter));
  }

  // Select initial command source. Note that the select function calls on_change_source.
  {
    const auto initial_source = declare_parameter<std::string>("initial_source");
    if (!selector_->select(initial_source)) {
      throw std::invalid_argument("invalid initial source: " + initial_source);
    }
    selector_->select_builtin_source(builtin);
  }

  const auto period = rclcpp::Rate(declare_parameter<double>("rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void ControlCmdGate::on_timer()
{
  selector_->update();
}

void ControlCmdGate::on_select_source(
  const SelectCommandSource::Request::SharedPtr req,
  const SelectCommandSource::Response::SharedPtr res)
{
  const auto result = selector_->select(req->source);
  if (result) {
    RCLCPP_INFO_STREAM(get_logger(), "changed command source: " << req->source);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "unknown command source: " << req->source);
  }
  res->status.success = result;
}

}  // namespace autoware::control_command_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_command_gate::ControlCmdGate)
