// Copyright 2023 Tier IV, Inc.
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

#ifndef TIER4_AUTOWARE_UTILS__GEOGRAPHY__HEIGHT_HPP_
#define TIER4_AUTOWARE_UTILS__GEOGRAPHY__HEIGHT_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace tier4_autoware_utils
{

typedef double (*HeightConversionFunction)(
  const double height, [[maybe_unused]] const double latitude,
  [[maybe_unused]] const double longitude);
double convert_wgs84_to_egm2008(
  const double height, [[maybe_unused]] const double latitude,
  [[maybe_unused]] const double longitude);
double convert_egm2008_to_wgs84(
  const double height, [[maybe_unused]] const double latitude,
  [[maybe_unused]] const double longitude);
double convert_height(
  const double height, const double latitude, const double longitude,
  const std::string & source_vertical_datum, const std::string & target_vertical_datum);

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__GEOGRAPHY__HEIGHT_HPP_
