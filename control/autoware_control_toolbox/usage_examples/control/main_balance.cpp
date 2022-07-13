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

#include "autoware_control_toolbox.hpp"

#include <vector>

int main()
{
	double Td    = 0.11;  // time delay in seconds.
	size_t order = 3;  // order of the Pade approximation.

	auto tf_delay = ns_control_toolbox::pade(Td, order);
	tf_delay.print();

	// Define Ts
	double Ts{ 0.1 };
	auto   ss_sys = ns_control_toolbox::tf2ss(tf_delay, Ts);
	ss_sys.print();

	// COMPARE with MATLAB RESULTS.
	double tau_steer{ 0.3 };
	double wheelbase{ 2.9 };  // L in vehicle model.

	ns_control_toolbox::tf_factor m_den1{{ wheelbase, 0, 0 }};  // L*s^2
	ns_control_toolbox::tf_factor m_den2{{ tau_steer, 1 }};     // (tau*s + 1)
	auto                          den_tf_factor = m_den1 * m_den2;

	ns_control_toolbox::tf Gey({ 1. }, den_tf_factor(), 100., 1.);  // num, den, num constant, den constant
	Gey.print();

	// Qfilter
	double dt    = 1. / 40;
	double frq   = 20.;
	double wc    = 2 * M_PI * frq;
	double tau_q = 1 / wc;

	ns_control_toolbox::tf_factor den_q{{ tau_q, 1 }};
	den_q.power(3);

	auto Qtf = ns_control_toolbox::tf({ 1 }, { den_q() });
	Qtf.print();

	Gey.inv();
	Gey.print();

	auto QGinv = Qtf * Gey;
	QGinv.print();

	auto QGinvss = ns_control_toolbox::tf2ss(QGinv, dt);

	ns_utils::print("Balanced System Matrices");
	QGinvss.print();

	// Balanced vehicle model.
	Gey.inv();
	Gey.print();

	auto Gey_ss = ns_control_toolbox::tf2ss(Gey, dt);

	Gey_ss.print();

	// PERMUTATION TEST

	return 0;
}