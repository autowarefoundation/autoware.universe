//
// Created by ali on 1/06/22.
//

#include "autoware_control_toolbox.hpp"

int main()
{
	double tfinal = 10.;
	double dt = 1. / 40.;
	auto time_vec = ns_control_toolbox::make_time_signal(dt, tfinal);

	ns_utils::print("Time points");
	ns_eigen_utils::printEigenMat(time_vec);


	// Creating a sinusoidal points.
	double frequency_hz = 1;

	auto sin_signal = ns_control_toolbox::make_sinus_signal(time_vec, frequency_hz);
	ns_utils::print("Sinusoidal Signal");
	ns_eigen_utils::printEigenMat(sin_signal);

	// test triangle wave

	auto triangle_wave = ns_control_toolbox::make_triangle_signal(time_vec, frequency_hz);
	ns_eigen_utils::printEigenMat(triangle_wave);
	return 0;
}