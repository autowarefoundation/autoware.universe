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

#ifndef CORE__EXPR_HPP_
#define CORE__EXPR_HPP_

#include "types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace system_diagnostic_graph
{

class BaseExpr
{
public:
  static std::unique_ptr<BaseExpr> create(const std::string & type);
  virtual ~BaseExpr() = default;
  virtual DiagnosticLevel exec(const std::vector<DiagnosticLevel> & levels) const = 0;
};

class ConstExpr : public BaseExpr
{
public:
  explicit ConstExpr(const DiagnosticLevel level) : level_(level) {}
  DiagnosticLevel exec(const std::vector<DiagnosticLevel> & levels) const override;

private:
  DiagnosticLevel level_;
};

class AllExpr : public BaseExpr
{
public:
  DiagnosticLevel exec(const std::vector<DiagnosticLevel> & levels) const override;
};

class AnyExpr : public BaseExpr
{
public:
  DiagnosticLevel exec(const std::vector<DiagnosticLevel> & levels) const override;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__EXPR_HPP_
