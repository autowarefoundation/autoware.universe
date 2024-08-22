// Copyright 2024 The Autoware Foundation
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

#ifndef UNIT_CONVERSIONS_HPP_
#define UNIT_CONVERSIONS_HPP_

#include <cstdint>
#include <type_traits>

namespace autoware::component_monitor::unit_conversions
{
template <typename T>
std::uint64_t kib_to_bytes(T kibibytes)
{
  static_assert(std::is_arithmetic<T>::value, "Template parameter must be a numeric type");
  return static_cast<std::uint64_t>(kibibytes * 1024);
}

}  // namespace autoware::component_monitor::unit_conversions

#endif  // UNIT_CONVERSIONS_HPP_
