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
#include <string>
#include <unordered_map>
#include <vector>

namespace system_diagnostic_graph
{

struct ConfigData
{
  ConfigData load(YAML::Node yaml);
  ConfigData type(const std::string & name) const;
  ConfigData node(const size_t index) const;

  std::string take_text(const std::string & name);
  std::string take_text(const std::string & name, const std::string & fail);
  std::vector<YAML::Node> take_list(const std::string & name);

  std::string file;
  std::string mark;
  std::unordered_map<std::string, YAML::Node> object;
};

struct UnitConfig
{
  using SharedPtr = std::shared_ptr<UnitConfig>;
  std::string type;
  std::string path;
  std::vector<UnitConfig::SharedPtr> children;
  ConfigData data;
};

struct FileConfig
{
  using SharedPtr = std::shared_ptr<FileConfig>;
  std::string package_name;
  std::string package_path;
  std::string path;
  std::vector<FileConfig::SharedPtr> files;
  std::vector<UnitConfig::SharedPtr> nodes;
  ConfigData data;
};

struct RootConfig
{
  std::vector<FileConfig::SharedPtr> files;
  std::vector<UnitConfig::SharedPtr> nodes;
};

RootConfig load_config_root(const std::string & path);

}  // namespace system_diagnostic_graph

#endif  // CORE__CONFIG_HPP_
