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

#ifndef CORE__UNITS_HPP_
#define CORE__UNITS_HPP_

#include "config.hpp"
#include "types.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class BaseUnit
{
public:
  struct NodeDict
  {
    std::unordered_map<UnitConfig::SharedPtr, BaseUnit *> configs;
    std::unordered_map<std::string, BaseUnit *> paths;
  };
  struct NodeData
  {
    DiagnosticLevel level;
    std::vector<std::pair<BaseUnit *, bool>> links;
  };
  using UniquePtr = std::unique_ptr<BaseUnit>;
  using UniquePtrList = std::vector<std::unique_ptr<BaseUnit>>;

  explicit BaseUnit(const std::string & path);
  virtual ~BaseUnit() = default;
  virtual void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) = 0;
  virtual void eval() = 0;

  DiagnosticNode report(const rclcpp::Time & stamp);
  auto level() const { return level_; }
  auto links() const { return links_; }
  auto path() const { return path_; }

  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

protected:
  virtual DiagnosticNode update(const rclcpp::Time & stamp) = 0;
  DiagnosticLevel level_;
  std::vector<std::pair<BaseUnit *, bool>> links_;
  std::string path_;

private:
  size_t index_;
};

class DiagUnit : public BaseUnit
{
public:
  using BaseUnit::BaseUnit;
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;
  void eval() override;

  std::string name() const { return name_; }
  void callback(const DiagnosticStatus & status, const rclcpp::Time & stamp);

private:
  DiagnosticNode update(const rclcpp::Time & stamp) override;
  double timeout_;
  std::optional<std::pair<DiagnosticStatus, rclcpp::Time>> diagnostics_;
  std::string name_;
};

class AndUnit : public BaseUnit
{
public:
  AndUnit(const std::string & path, bool short_circuit);
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;
  void eval() override;

private:
  DiagnosticNode update(const rclcpp::Time & stamp) override;
  bool short_circuit_;
};

class OrUnit : public BaseUnit
{
public:
  using BaseUnit::BaseUnit;
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;
  void eval() override;

private:
  DiagnosticNode update(const rclcpp::Time & stamp) override;
};

class DebugUnit : public BaseUnit
{
public:
  DebugUnit(const std::string & path, const DiagnosticLevel level);
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;
  void eval() override;

private:
  DiagnosticNode update(const rclcpp::Time & stamp) override;
  DiagnosticLevel const_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__UNITS_HPP_
