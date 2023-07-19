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

#ifndef LIB__GRAPH_HPP_
#define LIB__GRAPH_HPP_

#include "node.hpp"
#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace system_diagnostic_graph
{

class DiagGraph
{
public:
  DiagnosticGraph create(const std::string & file);
  DiagnosticArray report(const rclcpp::Time & stamp);
  void callback(const DiagnosticArray & array);
  void debug();

private:
  void topological_sort();
  DiagGraphData data_;

  struct NodeWithIndexLink
  {
    DiagNode * node;
    std::vector<uint32_t> links;
  };
  std::vector<NodeWithIndexLink> index_nodes_;
};

}  // namespace system_diagnostic_graph

#endif  // LIB__GRAPH_HPP_
