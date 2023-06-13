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
#include "types.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class DiagGraphInit;

class DiagNode
{
public:
  virtual DiagnosticNode report() = 0;
  virtual std::vector<DiagNode *> links() const = 0;
  virtual std::string name() const = 0;

protected:
  DiagnosticLevel level_;
};

class DiagUnit : public DiagNode
{
public:
  using KeyType = std::string;
  explicit DiagUnit(const KeyType & key);
  DiagnosticNode report() override { return DiagnosticNode(); }
  DiagDebugData debug();
  void update();
  void create(DiagGraphInit & graph, const UnitConfig & config);

  std::vector<DiagNode *> links() const override { return links_; }
  std::string name() const override { return "Unit[" + key_ + "]"; }

private:
  const KeyType key_;
  std::vector<DiagNode *> links_;
};

class DiagLeaf : public DiagNode
{
public:
  using KeyType = std::pair<std::string, std::string>;
  explicit DiagLeaf(const KeyType & key);
  DiagnosticNode report() override;
  DiagDebugData debug();
  void update(const DiagnosticStatus & status);

  std::vector<DiagNode *> links() const override { return {}; }
  std::string name() const override { return "Diag[" + key_.first + "]"; }

private:
  const KeyType key_;
};

struct DiagGraphData
{
  std::vector<std::unique_ptr<DiagUnit>> unit_list;
  std::vector<std::unique_ptr<DiagLeaf>> leaf_list;
  std::map<DiagUnit::KeyType, DiagUnit *> unit_dict;
  std::map<DiagLeaf::KeyType, DiagLeaf *> leaf_dict;

  DiagUnit * make_unit(const std::string & name);
  DiagUnit * find_unit(const std::string & name);
  DiagLeaf * make_leaf(const std::string & name, const std::string & hardware);
  DiagLeaf * find_leaf(const std::string & name, const std::string & hardware);
};

class DiagGraphInit
{
public:
  explicit DiagGraphInit(DiagGraphData & data) : data_(data) {}
  DiagNode * get(const LinkConfig & link);

private:
  DiagGraphData & data_;
};

}  // namespace system_diagnostic_graph

#endif  // LIB__NODE_HPP_
