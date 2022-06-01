//
// Created by ali on 1/06/22.
//

#include "autoware_control_toolbox.hpp"

int main()
{
	auto time_vec = ns_control_toolbox::make_time_signal(0.1, 10);
	ns_eigen_utils::printEigenMat(time_vec);

	double frequency_hz = 2;
	// test triangle wave
	auto triangle_wave = ns_control_toolbox::make_triangle_signal(time_vec, frequency_hz);
	ns_eigen_utils::printEigenMat(triangle_wave);
	return 0;
}