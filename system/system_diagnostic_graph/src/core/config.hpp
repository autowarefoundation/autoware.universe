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

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace system_diagnostic_graph
{

struct ConfigError : public std::runtime_error
{
  using runtime_error::runtime_error;
};

struct NodeConfig_
{
  std::string file;
  std::string path;
  YAML::Node yaml;
};

struct FileConfig_
{
  std::string path;
  std::vector<std::shared_ptr<FileConfig_>> files;
  std::vector<std::shared_ptr<NodeConfig_>> nodes;
};

using NodeConfig = std::shared_ptr<NodeConfig_>;
using FileConfig = std::shared_ptr<FileConfig_>;
using ConfigDict = std::unordered_map<std::string, YAML::Node>;

struct ExprConfig
{
  std::string type;
  ConfigDict dict;
};

ConfigError create_error(const FileConfig & config, const std::string & message);
ConfigError create_error(const NodeConfig & config, const std::string & message);
std::vector<NodeConfig> load_config_file(const std::string & path);

FileConfig parse_config_file(const std::string & path);
FileConfig parse_config_path(const std::string & path, const FileConfig & scope);
FileConfig parse_config_path(YAML::Node yaml, const FileConfig & scope);
NodeConfig parse_config_node(YAML::Node yaml, const FileConfig & scope);
ExprConfig parse_expr_object(YAML::Node yaml);

std::string take_expr_text(ConfigDict & dict, const std::string & name);
std::string take_expr_text(ConfigDict & dict, const std::string & name, const std::string & fail);
std::vector<YAML::Node> take_expr_list(ConfigDict & dict, const std::string & name);

}  // namespace system_diagnostic_graph

#endif  // CORE__CONFIG_HPP_
