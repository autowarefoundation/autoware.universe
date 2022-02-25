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

#ifndef SAMPLER_COMMON__CONSTRAINTS__SOFT_CONSTRAINT_HPP
#define SAMPLER_COMMON__CONSTRAINTS__SOFT_CONSTRAINT_HPP

#include "sampler_common/structures.hpp"

#include <functional>
#include <vector>

namespace sampler_common::constraints
{
/// @brief calculate the cost of the given path
void calculateCost(Path & path, const Constraints & constraints);
}  // namespace sampler_common::constraints

#endif  // SAMPLER_COMMON__CONSTRAINTS__SOFT_CONSTRAINT_HPP
