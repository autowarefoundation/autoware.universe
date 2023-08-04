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

#ifndef CORE__CONFIG_HPP_
#define CORE__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace system_diagnostic_graph
{

struct ConfigError : public std::runtime_error
{
  using runtime_error::runtime_error;
};

struct LinkConfig
{
  bool is_unit_type;
  std::string name;
  std::string hardware;
};

struct ExprConfig
{
  std::string type;
  std::vector<LinkConfig> list;
  YAML::Node yaml;
};

struct UnitConfig
{
  std::string name;
  std::string hint;
  ExprConfig expr;
};

std::vector<UnitConfig> load_config_file(const std::string & path);

}  // namespace system_diagnostic_graph

#endif  // CORE__CONFIG_HPP_
