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

#include "graph.hpp"
#include "node.hpp"
#include "types.hpp"

#include <iostream>
#include <unordered_map>

namespace system_diagnostic_graph
{

const std::unordered_map<DiagnosticLevel, std::string> level_names = {
  {DiagnosticStatus::OK, "OK"},
  {DiagnosticStatus::WARN, "WARN"},
  {DiagnosticStatus::ERROR, "ERROR"},
  {DiagnosticStatus::STALE, "STALE"}};

void DiagGraph::dump()
{
  std::cout << "============================== dump ==============================" << std::endl;
  for (const auto & diag : diags_) {
    diag.second->dump();
  }
}

void DiagLeaf::dump()
{
  std::cout << key_.first << " " << key_.second << " ";
  std::cout << level_names.at(level_) << " ";
  std::cout << std::endl;
}

}  // namespace system_diagnostic_graph
