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
#include <string>
#include <unordered_map>
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
  using UniquePtr = std::unique_ptr<BaseUnit>;
  using UniquePtrList = std::vector<std::unique_ptr<BaseUnit>>;

  explicit BaseUnit(const std::string & path);
  virtual ~BaseUnit() = default;
  virtual void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) = 0;

  std::string path() const { return path_; }
  std::vector<BaseUnit *> children() const { return children_; }

  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

  /*
  virtual void update(const rclcpp::Time & stamp) = 0;
  virtual DiagnosticNode report() const = 0;
  virtual DiagnosticLevel level() const = 0;
  */
  virtual void update(const rclcpp::Time & stamp) { (void)stamp; }
  virtual DiagnosticNode report() const { return DiagnosticNode(); }
  virtual DiagnosticLevel level() const { return DiagnosticStatus::OK; }

  /*
  virtual std::vector<BaseNode *> links() const = 0;
  */

protected:
  const std::string path_;
  size_t index_;
  std::vector<BaseUnit *> children_;
};

class DiagUnit : public BaseUnit
{
public:
  using BaseUnit::BaseUnit;
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;
  void callback(const DiagnosticStatus & status, const rclcpp::Time & stamp);
};

class LinkUnit : public BaseUnit
{
public:
  using BaseUnit::BaseUnit;
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;
};

class AndUnit : public BaseUnit
{
public:
  AndUnit(const std::string & path, bool short_circuit);
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;

private:
  bool short_circuit_;
};

class OrUnit : public BaseUnit
{
public:
  using BaseUnit::BaseUnit;
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;
};

class DebugUnit : public BaseUnit
{
public:
  DebugUnit(const std::string & path, const DiagnosticLevel level);
  void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) override;

private:
  DiagnosticLevel level_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__UNITS_HPP_
