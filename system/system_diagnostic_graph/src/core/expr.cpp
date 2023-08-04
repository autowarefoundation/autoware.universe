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

#include "expr.hpp"

#include "config.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace system_diagnostic_graph
{

std::unique_ptr<BaseExpr> BaseExpr::create(const std::string & type)
{
  if (type == "and") {
    return std::make_unique<AllExpr>();
  }
  if (type == "or") {
    return std::make_unique<AnyExpr>();
  }
  if (type == "debug-ok") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::OK);
  }
  if (type == "debug-warn") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::WARN);
  }
  if (type == "debug-error") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::ERROR);
  }
  if (type == "debug-stale") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::STALE);
  }
  throw ConfigError("unknown node type: " + type);
}

DiagnosticLevel ConstExpr::exec(const std::vector<DiagnosticLevel> &) const
{
  return level_;
}

DiagnosticLevel AllExpr::exec(const std::vector<DiagnosticLevel> & levels) const
{
  const auto level = *std::max_element(levels.begin(), levels.end());
  return std::min(level, DiagnosticStatus::ERROR);
}

DiagnosticLevel AnyExpr::exec(const std::vector<DiagnosticLevel> & levels) const
{
  const auto level = *std::min_element(levels.begin(), levels.end());
  return std::min(level, DiagnosticStatus::ERROR);
}

}  // namespace system_diagnostic_graph
