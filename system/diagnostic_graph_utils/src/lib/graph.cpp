// Copyright 2024 The Autoware Contributors
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

#include "diagnostic_graph_utils/graph.hpp"

namespace diagnostic_graph_utils
{

void DiagGraph::create(const DiagGraphStruct & msg)
{
  id_ = msg.id;
  for (const auto & node : msg.nodes) nodes_.push_back(std::make_unique<DiagNode>(node));
  for (const auto & diag : msg.diags) diags_.push_back(std::make_unique<DiagLeaf>(diag));
  for (const auto & link : msg.links) links_.push_back(std::make_unique<DiagLink>(link));

  const auto get_child = [this](bool is_leaf, size_t index) -> DiagUnit * {
    if (is_leaf) {
      return diags_.at(index).get();
    } else {
      return nodes_.at(index).get();
    }
  };

  for (const auto & link : msg.links) {
    DiagNode * parent = nodes_.at(link.parent).get();
    DiagUnit * child = get_child(link.is_leaf, link.child);
    parent->add_children(child);
  }
}

bool DiagGraph::update(const DiagGraphStatus & msg)
{
  if (id_ != msg.id) return false;
  for (size_t i = 0; i < msg.nodes.size(); ++i) nodes_[i]->update(msg.nodes[i]);
  for (size_t i = 0; i < msg.diags.size(); ++i) diags_[i]->update(msg.diags[i]);
  for (size_t i = 0; i < msg.links.size(); ++i) links_[i]->update(msg.links[i]);
  return true;
}

}  // namespace diagnostic_graph_utils
