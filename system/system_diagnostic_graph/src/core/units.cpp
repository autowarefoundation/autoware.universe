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
#include <vector>

namespace system_diagnostic_graph
{

auto resolve(const BaseUnit::NodeDict & dict, const std::vector<UnitConfig::SharedPtr> & children)
{
  std::vector<BaseUnit *> result;
  for (const auto & child : children) {
    result.push_back(dict.configs.at(child));
  }
  return result;
}

auto resolve(const BaseUnit::NodeDict & dict, const std::string & path)
{
  std::vector<BaseUnit *> result;
  result.push_back(dict.paths.at(path));
  return result;
}

BaseUnit::BaseUnit(const std::string & path) : path_(path)
{
  index_ = 0;
}

void DiagUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  (void)config;
  (void)dict;
}

void DiagUnit::callback(const DiagnosticStatus & status, const rclcpp::Time & stamp)
{
  (void)status;
  (void)stamp;
}

void LinkUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  children_ = resolve(dict, config->data.take_text("link"));
}

AndUnit::AndUnit(const std::string & path, bool short_circuit) : BaseUnit(path)
{
  short_circuit_ = short_circuit;
}

void AndUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  children_ = resolve(dict, config->children);
}

void OrUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  children_ = resolve(dict, config->children);
}

DebugUnit::DebugUnit(const std::string & path, const DiagnosticLevel level) : BaseUnit(path)
{
  level_ = level;
}

void DebugUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  (void)config;
  (void)dict;
}

}  // namespace system_diagnostic_graph
