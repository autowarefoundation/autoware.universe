// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_STATE_MONITOR__CONFIG_HPP_
#define AUTOWARE_STATE_MONITOR__CONFIG_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

struct TopicConfig
{
  explicit TopicConfig(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
    const std::string & namespace_prefix, const std::string & name)
  : module(interface->declare_parameter(namespace_prefix + ".module").get<std::string>()),
    name(name),
    type(interface->declare_parameter(namespace_prefix + ".type").get<std::string>()),
    timeout(interface->declare_parameter(namespace_prefix + ".timeout").get<double>()),
    warn_rate(interface->declare_parameter(namespace_prefix + ".warn_rate").get<double>())
  {
  }

  std::string module;
  std::string name;
  std::string type;
  double timeout;
  double warn_rate;
};

struct ParamConfig
{
  explicit ParamConfig(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
    const std::string & namespace_prefix, const std::string & name)
  : module(interface->declare_parameter(namespace_prefix + ".module").get<std::string>()),
    name(name)
  {
  }

  std::string module;
  std::string name;
};

struct TfConfig
{
  explicit TfConfig(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
    const std::string & namespace_prefix, const std::string & name)
  : module(interface->declare_parameter(namespace_prefix + ".module").get<std::string>()),
    from(interface->declare_parameter(namespace_prefix + ".from").get<std::string>()),
    to(interface->declare_parameter(namespace_prefix + ".to").get<std::string>()),
    timeout(interface->declare_parameter(namespace_prefix + ".timeout").get<double>())
  {
  }

  std::string module;
  std::string from;
  std::string to;
  double timeout;
};

struct TopicStats
{
  rclcpp::Time checked_time;
  std::vector<TopicConfig> ok_list;
  std::vector<TopicConfig> non_received_list;
  std::vector<std::pair<TopicConfig, rclcpp::Time>> timeout_list;  // pair<TfConfig, last_received>
  std::vector<std::pair<TopicConfig, double>> slow_rate_list;   // pair<TfConfig, rate>
};

struct ParamStats
{
  rclcpp::Time checked_time;
  std::vector<ParamConfig> ok_list;
  std::vector<ParamConfig> non_set_list;
};

struct TfStats
{
  rclcpp::Time checked_time;
  std::vector<TfConfig> ok_list;
  std::vector<TfConfig> non_received_list;
  std::vector<std::pair<TfConfig, rclcpp::Time>> timeout_list;  // pair<TfConfig, last_received>
};

#endif  // AUTOWARE_STATE_MONITOR__CONFIG_HPP_
