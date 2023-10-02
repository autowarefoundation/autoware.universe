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

#ifndef CORE__NODE_HPP_
#define CORE__NODE_HPP_

#include "config.hpp"
#include "debug.hpp"
#include "types.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class BaseNode
{
public:
  explicit BaseNode(const std::string & path);
  virtual ~BaseNode() = default;
  virtual void create(ConfigObject & config, ExprInit & exprs) = 0;
  virtual void update(const rclcpp::Time & stamp) = 0;
  virtual DiagnosticNode report() const = 0;
  virtual DiagnosticLevel level() const = 0;
  virtual DiagDebugData debug() const = 0;
  virtual std::vector<BaseNode *> links() const = 0;

  std::string path() const { return path_; }

  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

protected:
  const std::string path_;
  size_t index_;
};

class UnitNode : public BaseNode
{
public:
  explicit UnitNode(const std::string & path);
  ~UnitNode() override;
  void create(ConfigObject & config, ExprInit & exprs) override;
  void update(const rclcpp::Time & stamp) override;
  DiagnosticNode report() const override;
  DiagnosticLevel level() const override;
  DiagDebugData debug() const override;
  std::vector<BaseNode *> links() const override;

private:
  std::unique_ptr<BaseExpr> expr_;
  std::vector<DiagnosticLink> links_;
  DiagnosticLevel level_;
};

class DiagNode : public BaseNode
{
public:
  DiagNode(const std::string & path, ConfigObject & config);
  void create(ConfigObject & config, ExprInit & exprs) override;
  void update(const rclcpp::Time & stamp) override;
  DiagnosticNode report() const override;
  DiagnosticLevel level() const override;
  DiagDebugData debug() const override;
  std::vector<BaseNode *> links() const override { return {}; }
  std::pair<std::string, std::string> key() const;

  void callback(const DiagnosticStatus & status, const rclcpp::Time & stamp);

private:
  double timeout_;
  std::optional<rclcpp::Time> time_;
  std::string name_;
  std::string hardware_;
  DiagnosticStatus status_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__NODE_HPP_
