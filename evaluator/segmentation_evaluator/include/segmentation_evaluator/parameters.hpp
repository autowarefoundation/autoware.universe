// Copyright 2021 Tier IV, Inc.
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

#ifndef SEGMENTATION_EVALUATOR__PARAMETERS_HPP_
#define SEGMENTATION_EVALUATOR__PARAMETERS_HPP_

#include "segmentation_evaluator/metrics/metric.hpp"

#include <array>

namespace segmentation_diagnostics
{
/**
 * @brief Enumeration of trajectory metrics
 */
struct Parameters
{
  std::array<bool, static_cast<size_t>(Metric::SIZE)> metrics{};  // default values to false
};  // struct Parameters

}  // namespace localiaztion_diagnostics

#endif  // SEGMENTATION_EVALUATOR__PARAMETERS_HPP_
