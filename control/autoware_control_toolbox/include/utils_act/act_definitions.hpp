/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE_CONTROL_TOOLBOX_ACT_DEFINITIONS_HPP
#define AUTOWARE_CONTROL_TOOLBOX_ACT_DEFINITIONS_HPP

#include <limits>

namespace ns_control_toolbox
{
	constexpr auto   EPS   = std::numeric_limits<double>::epsilon();
	constexpr double RADIX = 2.; // used in the balance.hpp
} // namespace ns_control_toolbox

#endif //AUTOWARE_CONTROL_TOOLBOX_ACT_DEFINITIONS_HPP
