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

class ErrorHint
{
public:
  explicit ErrorHint(const std::string & file);
  operator std::string() const;
  ConfigError create_error(const std::string & message) const;

private:
  std::string file_;
};

ErrorHint::ErrorHint(const std::string & file)
{
  file_ = file;
}

ErrorHint::operator std::string() const
{
  return file_;
}

ConfigError ErrorHint::create_error(const std::string & message) const
{
  return ConfigError(message + ": " + file_);
}

ConfigNode parse_unit(YAML::Node unit, ErrorHint hint)
{
  if (!unit.IsMap()) {
    throw hint.create_error("unit object is not a dict");
  }
  if (!unit["name"]) {
    throw hint.create_error("unit object has no name");
  }
  const auto name = unit["name"].as<std::string>();
  return ConfigNode{unit, name, hint};
}

std::vector<ConfigNode> parse_file(YAML::Node file, ErrorHint hint)
{
  if (!file.IsMap()) {
    throw hint.create_error("file object is not a dict");
  }
  const auto package_name = file["package"].as<std::string>();
  const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
  return load_config_file(package_path + "/" + file["path"].as<std::string>());
}

std::vector<ConfigNode> parse_units(YAML::Node units, ErrorHint hint)
{
  if (!units.IsDefined() || units.IsNull()) {
    return {};
  }
  if (!units.IsSequence()) {
    throw hint.create_error("units section is not a list");
  }
  std::vector<ConfigNode> result;
  for (const auto & unit : units) {
    result.push_back(parse_unit(unit, hint));
  }
  return result;
}

std::vector<ConfigNode> parse_files(YAML::Node files, ErrorHint hint)
{
  if (!files.IsDefined() || files.IsNull()) {
    return {};
  }
  if (!files.IsSequence()) {
    throw hint.create_error("files section is not a list");
  }
  std::vector<ConfigNode> result;
  for (const auto & file : files) {
    const auto units = parse_file(file, hint);
    result.insert(result.end(), units.begin(), units.end());
  }
  return result;
}

std::vector<ConfigNode> load_config_file(const std::string & path)
{
  const auto hint = ErrorHint(path);
  if (!std::filesystem::exists(path)) {
    throw hint.create_error("config file is not exist");
  }
  const auto yaml = YAML::LoadFile(path);
  if (!yaml.IsMap()) {
    throw hint.create_error("config file is not a dict");
  }
  const auto units = parse_units(yaml["units"], hint);
  const auto files = parse_files(yaml["files"], hint);

  std::vector<ConfigNode> result;
  result.insert(result.end(), units.begin(), units.end());
  result.insert(result.end(), files.begin(), files.end());
  return result;
}

}  // namespace system_diagnostic_graph
