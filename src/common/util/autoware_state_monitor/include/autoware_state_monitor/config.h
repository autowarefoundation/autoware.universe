/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#pragma once

#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

struct TopicConfig
{
  explicit TopicConfig(XmlRpc::XmlRpcValue value)
  : module(static_cast<std::string>(value["module"])),
    name(static_cast<std::string>(value["name"])),
    timeout(static_cast<double>(value["timeout"])),
    warn_rate(static_cast<double>(value["warn_rate"]))
  {
  }

  std::string module;
  std::string name;
  double timeout;
  double warn_rate;
};

struct ParamConfig
{
  explicit ParamConfig(XmlRpc::XmlRpcValue value)
  : module(static_cast<std::string>(value["module"])), name(static_cast<std::string>(value["name"]))
  {
  }

  std::string module;
  std::string name;
};

struct TfConfig
{
  explicit TfConfig(XmlRpc::XmlRpcValue value)
  : module(static_cast<std::string>(value["module"])),
    from(static_cast<std::string>(value["from"])),
    to(static_cast<std::string>(value["to"])),
    timeout(static_cast<double>(value["timeout"]))
  {
  }

  std::string module;
  std::string from;
  std::string to;
  double timeout;
};

struct TopicStats
{
  ros::Time checked_time;
  std::vector<TopicConfig> non_received_list;
  std::vector<std::pair<TopicConfig, ros::Time>> timeout_list;  // pair<TfConfig, last_received>
  std::vector<std::pair<TopicConfig, double>> slow_rate_list;   // pair<TfConfig, rate>
};

struct ParamStats
{
  ros::Time checked_time;
  std::vector<ParamConfig> non_set_list;
};

struct TfStats
{
  ros::Time checked_time;
  std::vector<TfConfig> non_received_list;
  std::vector<std::pair<TfConfig, ros::Time>> timeout_list;  // pair<TfConfig, last_received>
};
