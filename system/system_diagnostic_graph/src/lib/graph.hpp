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

#include <map>
#include <memory>
#include <vector>

namespace system_diagnostic_graph
{

class DiagGraph
{
public:
  DiagGraph();
  DiagnosticGraph report(const rclcpp::Time & stamp);
  void update(const DiagnosticArray & array);
  void dump();

private:
  std::vector<std::shared_ptr<DiagNode>> nodes_;
  std::map<DiagLeaf::Key, std::shared_ptr<DiagLeaf>> diags_;
};

}  // namespace system_diagnostic_graph

#endif  // LIB__GRAPH_HPP_
