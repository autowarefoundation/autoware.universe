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

#include <iostream>

namespace system_diagnostic_graph
{

void DiagGraph::dump()
{
  std::cout << "============================== dump ==============================" << std::endl;
  for (const auto & diag : diags_) {
    diag.second->dump();
  }
}

void DiagLeaf::dump()
{
  std::cout << key_.first << " " << key_.second << std::endl;
}

}  // namespace system_diagnostic_graph
