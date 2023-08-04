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

#ifndef LIB__NODE_HPP_
#define LIB__NODE_HPP_

#include "config.hpp"
#include "debug.hpp"
#include "expr.hpp"
#include "types.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class DiagGraphInit;

class BaseNode
{
public:
  virtual ~BaseNode() = default;
  virtual void update() = 0;
  virtual DiagnosticNode report() const = 0;
  virtual DiagDebugData debug() const = 0;
  virtual std::vector<BaseNode *> links() const = 0;
  virtual std::string name() const = 0;

  DiagnosticLevel level() const { return level_; }
  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

protected:
  size_t index_ = 0;
  DiagnosticLevel level_;
};

class UnitNode : public BaseNode
{
public:
  using KeyType = std::string;
  explicit UnitNode(const KeyType & key);
  DiagnosticNode report() const override;
  DiagDebugData debug() const override;
  void update() override;
  void create(DiagGraphInit & graph, const UnitConfig & config);

  std::vector<BaseNode *> links() const override { return links_; }
  std::string name() const override { return key_; }

private:
  const KeyType key_;
  std::vector<BaseNode *> links_;
  std::unique_ptr<BaseExpr> expr_;
};

class DiagNode : public BaseNode
{
public:
  using KeyType = std::pair<std::string, std::string>;
  explicit DiagNode(const KeyType & key);
  DiagnosticNode report() const override;
  DiagDebugData debug() const override;
  void update() override;
  void callback(const DiagnosticStatus & status);

  std::vector<BaseNode *> links() const override { return {}; }
  std::string name() const override { return key_.first; }

private:
  const KeyType key_;
  DiagnosticStatus status_;
};

struct DiagGraphData
{
  std::vector<std::unique_ptr<UnitNode>> unit_list;
  std::vector<std::unique_ptr<DiagNode>> leaf_list;
  std::map<UnitNode::KeyType, UnitNode *> unit_dict;
  std::map<DiagNode::KeyType, DiagNode *> leaf_dict;

  UnitNode * make_unit(const std::string & name);
  UnitNode * find_unit(const std::string & name);
  DiagNode * make_leaf(const std::string & name, const std::string & hardware);
  DiagNode * find_leaf(const std::string & name, const std::string & hardware);
};

class DiagGraphInit
{
public:
  explicit DiagGraphInit(DiagGraphData & data) : data_(data) {}
  BaseNode * get(const LinkConfig & link);

private:
  DiagGraphData & data_;
};

}  // namespace system_diagnostic_graph

#endif  // LIB__NODE_HPP_
