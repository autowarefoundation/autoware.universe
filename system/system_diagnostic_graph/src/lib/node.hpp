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

#include "types.hpp"

namespace system_diagnostic_graph
{

class DiagNode
{
public:
  DiagNode();
  virtual DiagnosticNode report() = 0;
};

class DiagLeaf : public DiagNode
{
public:
  explicit DiagLeaf(const DiagnosticStatus & status);
  DiagnosticNode report() override;
  void update(const DiagnosticStatus & status);
};

}  // namespace system_diagnostic_graph

#endif  // LIB__NODE_HPP_
