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

LinkConfig parse_link(YAML::Node yaml, ErrorHint hint)
{
  if (!yaml.IsMap()) {
    throw hint.create_error("link object is not a dict");
  }
  if (!yaml["type"]) {
    throw hint.create_error("link object has no type");
  }
  if (!yaml["name"]) {
    throw hint.create_error("link object has no name");
  }

  const auto type = yaml["type"].as<std::string>();
  const auto name = yaml["name"].as<std::string>();
  if (type == "unit") {
    return LinkConfig{true, name, ""};
  }
  if (type == "diag") {
    const auto hardware = yaml["hardware"].as<std::string>("");
    return LinkConfig{false, name, hardware};
  }
  throw hint.create_error("link object has invalid type");
}

std::vector<LinkConfig> parse_list(YAML::Node yaml, ErrorHint hint)
{
  if (!yaml.IsDefined() || yaml.IsNull()) {
    return {};
  }
  if (!yaml.IsSequence()) {
    throw hint.create_error("list object is not a list");
  }
  std::vector<LinkConfig> list;
  for (const auto & link : yaml) {
    list.push_back(parse_link(link, hint));
  }
  return list;
}

ExprConfig parse_expr(YAML::Node yaml, ErrorHint hint)
{
  if (!yaml.IsMap()) {
    throw hint.create_error("expr object is not a dict");
  }
  if (!yaml["type"]) {
    throw hint.create_error("expr object has no type");
  }
  const auto type = yaml["type"].as<std::string>();
  const auto list = parse_list(yaml["list"], hint);
  return ExprConfig{type, list, yaml};
}

UnitConfig parse_unit(YAML::Node yaml, ErrorHint hint)
{
  if (!yaml.IsMap()) {
    throw hint.create_error("unit object is not a dict");
  }
  if (!yaml["name"]) {
    throw hint.create_error("unit object has no name");
  }
  const auto name = yaml["name"].as<std::string>();
  const auto expr = parse_expr(yaml, hint);
  return UnitConfig{name, hint, expr};
}

std::vector<UnitConfig> parse_units(YAML::Node yaml, ErrorHint hint)
{
  if (!yaml.IsDefined() || yaml.IsNull()) {
    return {};
  }
  if (!yaml.IsSequence()) {
    throw hint.create_error("units section is not a list");
  }
  std::vector<UnitConfig> units;
  for (const auto & unit : yaml) {
    units.push_back(parse_unit(unit, hint));
  }
  return units;
}

std::vector<UnitConfig> parse_file(YAML::Node yaml, ErrorHint hint)
{
  if (!yaml.IsMap()) {
    throw hint.create_error("file object is not a dict");
  }
  const auto package_name = yaml["package"].as<std::string>();
  const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
  return load_config_file(package_path + "/" + yaml["path"].as<std::string>());
}

std::vector<UnitConfig> parse_files(YAML::Node yaml, ErrorHint hint)
{
  if (!yaml.IsDefined() || yaml.IsNull()) {
    return {};
  }
  if (!yaml.IsSequence()) {
    throw hint.create_error("files section is not a list");
  }
  std::vector<UnitConfig> files;
  for (const auto & file : yaml) {
    const auto units = parse_file(file, hint);
    files.insert(files.end(), units.begin(), units.end());
  }
  return files;
}

std::vector<UnitConfig> load_config_file(const std::string & path)
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

  std::vector<UnitConfig> result;
  result.insert(result.end(), units.begin(), units.end());
  result.insert(result.end(), files.begin(), files.end());
  return result;
}

}  // namespace system_diagnostic_graph
