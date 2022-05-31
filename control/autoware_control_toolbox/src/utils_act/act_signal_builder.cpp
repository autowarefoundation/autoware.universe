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

#include "utils_act/act_signal_builder.hpp"

Eigen::VectorXd ns_control_toolbox::make_time_signal(const double& dt, const double& final_time)
{
	Eigen::Index num_of_time_points = static_cast<Eigen::Index>(final_time / dt ) + 1;
	auto&& time_vector = Eigen::VectorXd::LinSpaced(num_of_time_points, 0., final_time);

	return time_vector;
}

Eigen::VectorXd ns_control_toolbox::make_sinus_signal(const Eigen::VectorXd& time_vec, const double& frequency_hz)
{

	double const w_ = 2 * M_PI * frequency_hz; // [rad/sec]
	auto sin_vec = Eigen::VectorXd(time_vec.unaryExpr([&](auto const& t)
	                                                  { return sin(w_ * t); }));

	return sin_vec;
}

Eigen::VectorXd ns_control_toolbox::make_square_signal(const Eigen::VectorXd& time_vec, const double& frequency_hz)
{
	double const w_ = 2 * M_PI * frequency_hz; // [rad/sec]
	auto sqr_vec = Eigen::VectorXd(time_vec.unaryExpr([&](auto const& t)
	                                                  {
		                                                  return sin(w_ * t) < 0 ? -1. : sin(w_ * t) == 0 ? 0. : 1.;
	                                                  }));
	return sqr_vec;
}

Eigen::VectorXd ns_control_toolbox::make_triangle_signal(const Eigen::VectorXd& time_vec, const double& frequency_hz)
{
	double const w_ = 2 * M_PI * frequency_hz; // [rad/sec]
	double half_period{ 1. };

	if (frequency_hz > 0)
	{
		half_period = 1. / frequency_hz / 2.;
	}

	// define a starting point.
	double y0{};
	auto trg_vec = Eigen::VectorXd(time_vec.unaryExpr([&](auto const& t)
	                                                  {
		                                                  auto slope =
				                                                  sin(w_ * t) < 0 ? -1. : sin(w_ * t) == 0 ? 0. : 1.;

		                                                  // use normalized time intervals.
		                                                  auto&& dt = std::fmod(t, half_period) / half_period;
		                                                  y0 += slope * dt;
		                                                  return y0;
	                                                  }));
	return trg_vec;
}
