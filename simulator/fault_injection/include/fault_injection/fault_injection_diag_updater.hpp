// Copyright 2024 TIER IV, Inc.
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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef FAULT_INJECTION__FAULT_INJECTION_DIAG_UPDATER_HPP_
#define FAULT_INJECTION__FAULT_INJECTION_DIAG_UPDATER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace fault_injection
{
class FaultInjectionDiagUpdater : public diagnostic_updater::DiagnosticTaskVector
{
public:
  template <class NodeT>
  explicit FaultInjectionDiagUpdater(NodeT node, double period = 1.0)
  : FaultInjectionDiagUpdater(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_parameters_interface(),
      node->get_node_timers_interface(), node->get_node_topics_interface(), period)
  {
  }

  FaultInjectionDiagUpdater(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface,
    double period = 1.0)
  : base_interface_(base_interface),
    timers_interface_(timers_interface),
    clock_(clock_interface->get_clock()),
    period_(rclcpp::Duration::from_nanoseconds(period * 1e9)),
    publisher_(rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      topics_interface, "/diagnostics", 1)),
    logger_(logging_interface->get_logger()),
    node_name_(base_interface->get_name())
  {
    period = parameters_interface
               ->declare_parameter("diagnostic_updater.period", rclcpp::ParameterValue(period))
               .get<double>();
    period_ = rclcpp::Duration::from_nanoseconds(period * 1e9);

    reset_timer();
  }

  /**
   * \brief Returns the interval between updates.
   */
  auto getPeriod() const { return period_; }

  /**
   * \brief Sets the period as a rclcpp::Duration
   */
  void setPeriod(rclcpp::Duration period)
  {
    period_ = period;
    reset_timer();
  }

  /**
   * \brief Sets the period given a value in seconds
   */
  void setPeriod(double period) { setPeriod(rclcpp::Duration::from_nanoseconds(period * 1e9)); }

  /**
   * \brief Forces to send out an update for all known DiagnosticStatus.
   */
  void force_update() { update(); }

  /**
   * \brief Output a message on all the known DiagnosticStatus.
   *
   * Useful if something drastic is happening such as shutdown or a
   * self-test.
   *
   * \param lvl Level of the diagnostic being output.
   *
   * \param msg Status message to output.
   */
  void broadcast(int lvl, const std::string msg)
  {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

    const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
    for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
         iter != tasks.end(); iter++) {
      diagnostic_updater::DiagnosticStatusWrapper status;

      status.name = iter->getName();
      status.summary(lvl, msg);

      status_vec.push_back(status);
    }

    publish(status_vec);
  }

  void setHardwareIDf(const char * format, ...)
  {
    va_list va;
    const int kBufferSize = 1000;
    char buff[kBufferSize];  // @todo This could be done more elegantly.
    va_start(va, format);
    if (vsnprintf(buff, kBufferSize, format, va) >= kBufferSize) {
      RCLCPP_DEBUG(logger_, "Really long string in diagnostic_updater::setHardwareIDf.");
    }
    hwid_ = std::string(buff);
    va_end(va);
  }

  void setHardwareID(const std::string & hwid) { hwid_ = hwid; }

private:
  void reset_timer()
  {
    update_timer_ = rclcpp::create_timer(
      base_interface_, timers_interface_, clock_, period_,
      std::bind(&FaultInjectionDiagUpdater::update, this));
  }

  /**
   * \brief Causes the diagnostics to update if the inter-update interval
   * has been exceeded.
   */
  void update()
  {
    if (rclcpp::ok()) {
      std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

      std::unique_lock<std::mutex> lock(
        lock_);  // Make sure no adds happen while we are processing here.
      const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
      for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
           iter != tasks.end(); iter++) {
        diagnostic_updater::DiagnosticStatusWrapper status;

        status.name = iter->getName();
        status.level = 2;
        status.message = "No message was set";
        status.hardware_id = hwid_;

        iter->run(status);

        status_vec.push_back(status);
      }

      publish(status_vec);
    }
  }

  /**
   * Publishes a single diagnostic status.
   */
  void publish(diagnostic_msgs::msg::DiagnosticStatus & stat)
  {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;
    status_vec.push_back(stat);
    publish(status_vec);
  }

  /**
   * Publishes a vector of diagnostic statuses.
   */
  void publish(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec)
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.status = status_vec;
    msg.header.stamp = clock_->now();
    publisher_->publish(msg);
  }

  /**
   * Causes a placeholder DiagnosticStatus to be published as soon as a
   * diagnostic task is added to the Updater.
   */
  virtual void addedTaskCallback(DiagnosticTaskInternal & task)
  {
    diagnostic_updater::DiagnosticStatusWrapper stat;
    stat.name = task.getName();
    stat.summary(0, "Node starting up");
    publish(stat);
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_interface_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Logger logger_;

  std::string hwid_;
  std::string node_name_;
};
}  // namespace fault_injection

#endif  // FAULT_INJECTION__FAULT_INJECTION_DIAG_UPDATER_HPP_
