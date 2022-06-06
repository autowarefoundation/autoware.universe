//
// Created by ali on 21/2/22.
//

#include <fmt/core.h>
#include <cassert>

#include "autoware_control_toolbox.hpp"


int main()
{

	double Td = 0.01; // time delay in seconds.
	size_t order = 3; // order of the Pade approximation.

	auto tf_delay = ns_control_toolbox::pade(Td, order);
	tf_delay.print();

	auto ss_sys = ns_control_toolbox::tf2ss(tf_delay);
	ss_sys.print();

	// Test discretization and compare with Matlab.
	double Ts = 0.1;

	// ss_sys.discretisize(Ts);
	ss_sys.print_discrete_system();

	//
	ns_utils::print("Discretization with a given Ts when constructing");
	Ts = 0.15;
	auto sys_ss2 = ns_control_toolbox::tf2ss(tf_delay, Ts);
	sys_ss2.print();
	sys_ss2.print_discrete_system();


	// Edge case, when the delay = 0.
	Td = 0.0; // time delay in seconds.
	order = 3; // order of the Pade approximation.

	tf_delay = ns_control_toolbox::pade(Td, order);
	tf_delay.print();

	if (ns_utils::isEqual(Td, 0.0))
	{
		ns_utils::print("Must throw an error STATIC GAIN .........");
		ss_sys = ns_control_toolbox::tf2ss(tf_delay);
		ss_sys.print();
	}


	return 0;
}
