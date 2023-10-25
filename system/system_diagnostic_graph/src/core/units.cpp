// Copyright 2023 The Autoware Contributors
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

#include "units.hpp"

#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

auto resolve(const BaseUnit::NodeDict & dict, const std::vector<UnitConfig::SharedPtr> & links)
{
  std::vector<std::pair<BaseUnit *, bool>> result;
  for (const auto & link : links) {
    result.push_back(std::make_pair(dict.configs.at(link), true));
  }
  return result;
}

auto resolve(const BaseUnit::NodeDict & dict, const std::string & path)
{
  std::vector<std::pair<BaseUnit *, bool>> result;
  result.push_back(std::make_pair(dict.paths.at(path), true));
  return result;
}

BaseUnit::BaseUnit(const std::string & path) : path_(path)
{
  index_ = 0;
  level_ = DiagnosticStatus::STALE;
}

DiagnosticNode BaseUnit::report(const rclcpp::Time & stamp)
{
  const auto message = update(stamp);
  level_ = message.status.level;
  return message;
}

void DiagUnit::init(const UnitConfig::SharedPtr & config, const NodeDict &)
{
  timeout_ = 3.0;  // TODO(Takagi, Isamu): parameterize
  name_ = config->data.take_text("name", path_);
}

void DiagUnit::eval()
{
  if (diagnostics_) {
    level_ = diagnostics_.value().first.status.level;
  } else {
    level_ = DiagnosticStatus::STALE;
  }
}

void DiagUnit::callback(const DiagnosticStatus & status, const rclcpp::Time & stamp)
{
  diagnostics_ = std::make_pair(status, stamp);
}

DiagnosticNode DiagUnit::update(const rclcpp::Time & stamp)
{
  if (diagnostics_) {
    const auto updated = diagnostics_.value().second;
    const auto elapsed = (stamp - updated).seconds();
    if (timeout_ < elapsed) {
      diagnostics_ = std::nullopt;
    }
  }

  DiagnosticNode message;
  if (diagnostics_) {
    message.status = diagnostics_.value().first;
    message.status.name = path_;
  } else {
    message.status.level = DiagnosticStatus::STALE;
    message.status.name = path_;
  }
  return message;
}

AndUnit::AndUnit(const std::string & path, bool short_circuit) : BaseUnit(path)
{
  short_circuit_ = short_circuit;
}

void AndUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  links_ = resolve(dict, config->children);
}

DiagnosticNode AndUnit::update(const rclcpp::Time &)
{
  DiagnosticNode message;
  message.status.name = path_;
  return message;
}

void OrUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  links_ = resolve(dict, config->children);
}

DiagnosticNode OrUnit::update(const rclcpp::Time &)
{
  DiagnosticNode message;
  message.status.name = path_;
  return message;
}

DebugUnit::DebugUnit(const std::string & path, const DiagnosticLevel level) : BaseUnit(path)
{
  const_ = level;
}

void DebugUnit::init(const UnitConfig::SharedPtr &, const NodeDict &)
{
}

DiagnosticNode DebugUnit::update(const rclcpp::Time &)
{
  DiagnosticNode message;
  message.status.name = path_;
  return message;
}

}  // namespace system_diagnostic_graph
