//
// Created by ali on 21/2/22.
//

#include <fmt/core.h>
#include <cassert>

#include "autoware_control_toolbox.hpp"


int main()
{

	double Td    = 0.1; // time delay in seconds.
	size_t order = 3; // order of the Pade approximation.

	auto tf_delay = ns_control_toolbox::pade(Td, order);
	tf_delay.print();

	auto ss_sys = ns_control_toolbox::tf2ss(tf_delay);
	ss_sys.print();

	// Test discretization and compare with Matlab.
	double Ts = 0.1;
	ss_sys.discretisize(Ts);
	ss_sys.print_discrete_system();


	return 0;
}
