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

#include "config.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <iostream>
#include <string>

namespace system_diagnostic_graph
{

class ConfigMarker
{
public:
  explicit ConfigMarker(const std::string & path);
  ConfigError create_error(const std::string & message) const;

private:
  std::string path_;
};

ConfigMarker::ConfigMarker(const std::string & path)
{
  path_ = path;
}

ConfigError ConfigMarker::create_error(const std::string & message) const
{
  return ConfigError(message + ": " + path_);
}

YAML::Node parse_unit(YAML::Node unit, ConfigMarker marker)
{
  if (!unit.IsMap()) {
    throw marker.create_error("unit object is not a dict");
  }
  if (!unit["name"]) {
    throw marker.create_error("unit object has no name");
  }
  return unit;
}

std::vector<YAML::Node> parse_file(YAML::Node file, ConfigMarker marker)
{
  if (!file.IsMap()) {
    throw marker.create_error("file object is not a dict");
  }
  const auto package_name = file["package"].as<std::string>();
  const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
  return load_config_file(package_path + "/" + file["path"].as<std::string>());
}

std::vector<YAML::Node> parse_units(YAML::Node units, ConfigMarker marker)
{
  if (!units.IsDefined() || units.IsNull()) {
    return {};
  }
  if (!units.IsSequence()) {
    throw marker.create_error("units section is not a list");
  }
  std::vector<YAML::Node> result;
  for (const auto & unit : units) {
    result.push_back(parse_unit(unit, marker));
  }
  return result;
}

std::vector<YAML::Node> parse_files(YAML::Node files, ConfigMarker marker)
{
  if (!files.IsDefined() || files.IsNull()) {
    return {};
  }
  if (!files.IsSequence()) {
    throw marker.create_error("files section is not a list");
  }
  std::vector<YAML::Node> result;
  for (const auto & file : files) {
    const auto units = parse_file(file, marker);
    result.insert(result.end(), units.begin(), units.end());
  }
  return result;
}

std::vector<YAML::Node> load_config_file(const std::string & path)
{
  const auto marker = ConfigMarker(path);
  if (!std::filesystem::exists(path)) {
    throw marker.create_error("config file is not exist");
  }
  const auto yaml = YAML::LoadFile(path);
  if (!yaml.IsMap()) {
    throw marker.create_error("config file is not a dict");
  }
  const auto units = parse_units(yaml["units"], marker);
  const auto files = parse_files(yaml["files"], marker);

  std::vector<YAML::Node> result;
  result.insert(result.end(), units.begin(), units.end());
  result.insert(result.end(), files.begin(), files.end());
  return result;
}

}  // namespace system_diagnostic_graph
