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

#ifndef AUTOWARE_CONTROL_TOOLBOX_ACT_SIGNAL_BUILDER_HPP
#define AUTOWARE_CONTROL_TOOLBOX_ACT_SIGNAL_BUILDER_HPP

#include "act_utils.hpp"
#include "act_utils_eigen.hpp"

namespace ns_control_toolbox
{
	/**
	 * @brief Creates a time vector given a final time and time-step.
	 * */
	Eigen::VectorXd make_time_signal(double const& dt, double const& final_time);

	/**
	 * @brief Creates a sin wave vector given a time vector and frequency in Hz.
	 * */
	Eigen::VectorXd make_sinus_signal(Eigen::VectorXd const& time_vec, double const& frequency_hz);


	/**
	 * @brief Creates a square wave vector given a time vector and frequency in Hz.
	 * */
	Eigen::VectorXd make_square_signal(Eigen::VectorXd const& time_vec, double const& frequency_hz);

	/**
	 * @brief Creates a triangle wave vector given a time vector and frequency in Hz.
	 * */
	Eigen::VectorXd make_triangle_signal(Eigen::VectorXd const& time_vec, double const& frequency_hz);

} // namespace ns_control_toolbox

#endif //AUTOWARE_CONTROL_TOOLBOX_ACT_SIGNAL_BUILDER_HPP
