// Copyright 2022 Tier IV, Inc.
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

#ifndef SAMPLER_NODE__DEBUG_HPP_
#define SAMPLER_NODE__DEBUG_HPP_

#include <sampler_common/constraints/hard_constraint.hpp>

namespace sampler_node::debug
{
struct Debug
{
  sampler_common::constraints::NumberOfViolations violations;
};
}  // namespace sampler_node::debug

#endif  // SAMPLER_NODE__DEBUG_HPP_
