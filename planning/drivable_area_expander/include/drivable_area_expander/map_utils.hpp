// Copyright 2022 TIER IV, Inc.
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

#ifndef DRIVABLE_AREA_EXPANDER__MAP_UTILS_HPP_
#define DRIVABLE_AREA_EXPANDER__MAP_UTILS_HPP_

#include "drivable_area_expander/types.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace drivable_area_expander
{
/// @brief Extract uncrossable linestrings from the lanelet map
/// @param[in] lanelet_map lanelet map
/// @param[in] uncrossable_types types that cannot be crossed
/// @return the uncrossable linestrings
multilinestring_t extractUncrossableLines(
  const lanelet::LaneletMap & lanelet_map, const std::vector<std::string> & uncrossable_types);

/// @brief Determine if the given linestring has one of the given types
/// @param[in] ls linestring to check
/// @param[in] types type strings to check
/// @return true if the linestring has one of the given types
bool hasTypes(const lanelet::ConstLineString3d & ls, const std::vector<std::string> & types);
}  // namespace drivable_area_expander

#endif  // DRIVABLE_AREA_EXPANDER__MAP_UTILS_HPP_
