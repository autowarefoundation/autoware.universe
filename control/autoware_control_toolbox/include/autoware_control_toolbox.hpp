// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_CONTROL_TOOLBOX_HPP_
#define AUTOWARE_CONTROL_TOOLBOX_HPP_

#include <cstddef>
#include <iostream>
#include <array>
#include <utility>
#include <vector>
#include <sstream>
#include <iomanip>
#include <limits>
#include <boost/optional.hpp>

// Library headers
#include "visibility_control.hpp"
#include "utils_act/tf_algebra.hpp"
#include "utils_act/act_utils_eigen.hpp"
#include "utils_act/act_definitions.hpp"
#include "utils_act/balance.hpp"
#include "utils_act/transfer_functions.hpp"
#include "utils_act/state_space.hpp"
#include "utils_act/act_signal_builder.hpp"

template<typename T, typename std::enable_if<std::is_floating_point<T>::value, bool>::type = 1>
std::vector<T> operator*(std::vector<T> const& vec, T const& a)
{
	std::vector<T> temp{ vec };

	for (auto& x: temp)
	{
		x = x * a;
	}

	return temp;
}

template<typename T, typename std::enable_if<std::is_floating_point<T>::value, bool>::type = 1>
std::vector<T>& operator*=(std::vector<T>& vec, T const& a)
{
	for (auto& x: vec)
	{
		x = x * a;
	}

	return vec;
}

namespace ns_control_toolbox
{


} // namespace ns_control_toolbox
#endif // AUTOWARE_CONTROL_TOOLBOX_HPP_
