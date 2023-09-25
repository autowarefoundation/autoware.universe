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

#ifndef CORE__MANAGER_HPP_
#define CORE__MANAGER_HPP_

#include "graph.hpp"
#include "node.hpp"
#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace system_diagnostic_graph
{

struct Modes
{
  UnitNode * stop_mode;
  UnitNode * autonomous_mode;
  UnitNode * local_mode;
  UnitNode * remote_mode;
  UnitNode * emergency_stop_mrm;
  UnitNode * comfortable_stop_mrm;
  UnitNode * pull_over_mrm;
};

class GraphManager
{
public:
  void init(const std::string & file, const std::string & mode);
  void callback(const DiagnosticArray & array, const rclcpp::Time & stamp);
  void update(const rclcpp::Time & stamp);
  DiagnosticGraph create_graph_message() const;
  OperationModeAvailability create_modes_message() const;

  void debug();

private:
  Graph graph_;
  Modes modes_;
  rclcpp::Time stamp_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__MANAGER_HPP_
