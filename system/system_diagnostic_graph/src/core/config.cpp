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

#include "error.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// DEBUG
#include <iostream>

namespace system_diagnostic_graph
{

ConfigData ConfigData::load(YAML::Node yaml)
{
  if (!yaml.IsMap()) {
    throw ConfigError(std::string("TODO") + " is not a dict type");
  }
  for (const auto & kv : yaml) {
    object[kv.first.as<std::string>()] = kv.second;
  }
  return *this;
}

ConfigData ConfigData::type(const std::string & name) const
{
  ConfigData data;
  data.file = file;
  data.mark = name;
  return data;
}

ConfigData ConfigData::node(const size_t index) const
{
  ConfigData data;
  data.file = file;
  data.mark = mark + "-" + std::to_string(index);
  return data;
}

std::string ConfigData::take_text(const std::string & name)
{
  if (!object.count(name)) {
    throw ConfigError("object has no '" + name + "' field");
  }

  const auto yaml = object.at(name);
  object.erase(name);
  return yaml.as<std::string>();
}

std::string ConfigData::take_text(const std::string & name, const std::string & fail)
{
  if (!object.count(name)) {
    return fail;
  }

  const auto yaml = object.at(name);
  object.erase(name);
  return yaml.as<std::string>();
}

std::vector<YAML::Node> ConfigData::take_list(const std::string & name)
{
  if (!object.count(name)) {
    return std::vector<YAML::Node>();
  }

  const auto yaml = object.at(name);
  object.erase(name);

  if (!yaml.IsSequence()) {
    throw ConfigError("the '" + name + "' field is not a list type");
  }
  return std::vector<YAML::Node>(yaml.begin(), yaml.end());
}

template <class T>
void extend(std::vector<T> & u, const std::vector<T> & v)
{
  u.insert(u.end(), v.begin(), v.end());
}

template <class T>
auto enumerate(const std::vector<T> & v)
{
  std::vector<std::pair<size_t, T>> result;
  for (size_t i = 0; i < v.size(); ++i) {
    result.push_back(std::make_pair(i, v[i]));
  }
  return result;
}

UnitConfig::SharedPtr parse_node_config(const ConfigData & data)
{
  const auto node = std::make_shared<UnitConfig>();
  node->data = data;
  node->path = node->data.take_text("path", "");
  node->type = node->data.take_text("type");

  for (const auto & [index, yaml] : enumerate(node->data.take_list("list"))) {
    const auto child = data.node(index).load(yaml);
    node->children.push_back(parse_node_config(child));
  }
  return node;
}

FileConfig::SharedPtr parse_file_config(const ConfigData & data)
{
  const auto file = std::make_shared<FileConfig>();
  file->data = data;
  file->package_name = file->data.take_text("package");
  file->package_path = file->data.take_text("path");
  return file;
}

void dump_node(const UnitConfig & config, const std::string & indent = "")
{
  std::cout << indent << " - node: " << config.type << " (" << config.path << ")" << std::endl;
  for (const auto & child : config.children) dump_node(*child, indent + "  ");
}

void dump_file(const FileConfig & config)
{
  std::cout << "=================================================================" << std::endl;
  std::cout << config.path << std::endl;
  for (const auto & file : config.files) {
    std::cout << " - file: " << file->package_name << "/" << file->package_path << std::endl;
  }
  for (const auto & node : config.nodes) {
    dump_node(*node);
  }
}

void load_config_file(FileConfig & config, const ConfigData & parent)
{
  if (config.path.empty()) {
    const auto package_base = ament_index_cpp::get_package_share_directory(config.package_name);
    config.path = package_base + "/" + config.package_path;
  }

  if (!std::filesystem::exists(config.path)) {
    (void)parent;
    throw ConfigError("TODO");
  }

  ConfigData data;
  data.file = config.path.empty() ? config.package_path + "/" + config.package_path : "root";
  data.load(YAML::LoadFile(config.path));
  const auto files = data.take_list("files");
  const auto nodes = data.take_list("nodes");

  for (const auto & [index, yaml] : enumerate(files)) {
    const auto file = data.type("file").node(index).load(yaml);
    config.files.push_back(parse_file_config(file));
  }
  for (const auto & [index, yaml] : enumerate(nodes)) {
    const auto node = data.type("file").node(index).load(yaml);
    config.nodes.push_back(parse_node_config(node));
  }
}

void check_config_nodes(const std::vector<UnitConfig::SharedPtr> & nodes)
{
  std::unordered_map<std::string, size_t> path_count;
  for (const auto & node : nodes) {
    path_count[node->path] += 1;
  }

  path_count.erase("");
  for (const auto & [path, count] : path_count) {
    if (1 < count) {
      throw ConfigError("TODO: path conflict");
    }
  }
}

void resolve_link_nodes(std::vector<UnitConfig::SharedPtr> & nodes)
{
  std::vector<UnitConfig::SharedPtr> filtered;
  std::unordered_map<UnitConfig::SharedPtr, UnitConfig::SharedPtr> links;
  std::unordered_map<std::string, UnitConfig::SharedPtr> paths;

  for (const auto & node : nodes) {
    links[node] = node;
    paths[node->path] = node;
  }

  for (const auto & node : nodes) {
    if (node->type == "link" && node->path == "") {
      links[node] = paths.at(node->data.take_text("link"));
    } else {
      filtered.push_back(node);
    }
  }
  nodes = filtered;

  for (const auto & node : nodes) {
    for (auto & child : node->children) {
      child = links.at(child);
    }
  }
}

RootConfig load_config_root(const std::string & path)
{
  RootConfig config;
  config.files.push_back(std::make_shared<FileConfig>());
  config.files.back()->path = path;

  for (size_t i = 0; i < config.files.size(); ++i) {
    const auto & file = config.files[i];
    load_config_file(*file, {});
    extend(config.files, file->files);
  }

  for (const auto & file : config.files) {
    extend(config.nodes, file->nodes);
  }

  for (size_t i = 0; i < config.nodes.size(); ++i) {
    const auto & node = config.nodes[i];
    extend(config.nodes, node->children);
  }

  check_config_nodes(config.nodes);
  resolve_link_nodes(config.nodes);
  return config;
}

}  // namespace system_diagnostic_graph
