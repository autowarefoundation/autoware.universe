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

#include "graph.hpp"

#include "node.hpp"

#include <memory>
#include <utility>

namespace system_diagnostic_graph
{

UnitNode * Graph::make_unit(const std::string & name)
{
  const auto key = name;
  auto unit = std::make_unique<UnitNode>(key);
  units_[key] = unit.get();
  nodes_.emplace_back(std::move(unit));
  return units_[key];
}

UnitNode * Graph::find_unit(const std::string & name)
{
  const auto key = name;
  return units_.count(key) ? units_.at(key) : nullptr;
}

DiagNode * Graph::make_diag(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  auto diag = std::make_unique<DiagNode>(name, hardware);
  diags_[key] = diag.get();
  nodes_.emplace_back(std::move(diag));
  return diags_[key];
}

DiagNode * Graph::find_diag(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  return diags_.count(key) ? diags_.at(key) : nullptr;
}

}  // namespace system_diagnostic_graph
