// Copyright 2022 The Autoware Foundation
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


#ifndef COMMUNICATION_DELAY_COMPENSATOR__DELAY_COMPENSATION_UTILS_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__DELAY_COMPENSATION_UTILS_HPP

#include <utility>
#include <type_traits>
#include <limits>
#include <vector>
#include <filesystem>
#include <fstream>
#include <string>

#define GET_VARIABLE_NAME(Variable) (#Variable)
auto constexpr EPS = std::numeric_limits<double>::epsilon();

/**
 * @brief Fetching the underlying type from strongly typed Enum class.
 * */
template<typename E>
constexpr auto toUType(E e) noexcept
{
  return static_cast<std::underlying_type_t<E>>(e);
}

#endif  // COMMUNICATION_DELAY_COMPENSATOR__DELAY_COMPENSATION_UTILS_HPP
