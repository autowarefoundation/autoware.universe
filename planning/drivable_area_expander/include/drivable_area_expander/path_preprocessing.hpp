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

#ifndef DRIVABLE_AREA_EXPANDER__PATH_PREPROCESSING_HPP_
#define DRIVABLE_AREA_EXPANDER__PATH_PREPROCESSING_HPP_

#include "drivable_area_expander/types.hpp"

namespace drivable_area_expander
{
/// @brief calculate path index that is ahead of the given index by the given distance
/// @param[in] path path
/// @param[in] start_idx starting index
/// @param[in] max_length maximum length from start_idx to the returned index
/// @return path index ahead of the start_idx by at most the given length and duration
size_t calculateEndIndex(const Path & path, const size_t start_idx, const double max_length);

/// @brief downsample a path, reducing its number of points by the given factor
/// @param[in] path input path
/// @param[in] start_idx starting index of the input path
/// @param[in] end_idx ending index of the input path
/// @param[in] factor factor used for downsampling
/// @return downsampled path
Path downsamplePath(
  const Path & path, const size_t start_idx, const size_t end_idx, const int factor);
}  // namespace drivable_area_expander

#endif  // DRIVABLE_AREA_EXPANDER__PATH_PREPROCESSING_HPP_
