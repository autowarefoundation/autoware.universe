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

#include <utility>

namespace system_diagnostic_graph
{

DiagGraph::DiagGraph()
{
}

void DiagGraph::update(const DiagnosticArray & array)
{
  for (const auto & status : array.status) {
    const auto key = std::make_pair(status.name, status.hardware_id);
    if (!diags_.count(key)) {
      const auto diag = std::make_shared<DiagLeaf>(status);
      diags_.emplace(key, diag);
    }
    diags_.at(key)->update(status);
  }
}

}  // namespace system_diagnostic_graph
