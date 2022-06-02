//
// Created by ali on 17/2/22.
//

#include <fmt/core.h>
#include <cassert>
#include "utils_act/act_utils.hpp"
#include "autoware_control_toolbox.hpp"


int main()
{

	std::vector<double> num{ 1. };
	std::vector<double> den{ 5.039e-07, 0.00019, 0.02387, 1 };

	// With a num, den
	ns_control_toolbox::tf sys(num, den);

	// Print sys
	sys.print();

	// With a default constructor
	ns_utils::print("\n\n");
	ns_control_toolbox::tf sys_default;
	sys_default.print();

	// Inverse
	sys.inv();
	sys.print();

	// test construction from tf factors.
	ns_control_toolbox::tf_factor ntf1{{ 1, 0, 0 }};
	ns_control_toolbox::tf_factor dtf2{{ 1, 0, 0.2 }};

	auto tf_from_tf_factors = ns_control_toolbox::tf(ntf1, dtf2);
	tf_from_tf_factors.print();


	// Transfer function multiplication.
	ns_control_toolbox::tf tf1{{ 5 },
	                           { 1, 3 }};

	ns_control_toolbox::tf tf2{{ 2 },
	                           { 1, 3 }};

	auto tf3 = tf1 * tf2;

	tf3.print();

	// Test vector overloading.
	std::vector<double> ov1{ 1, 2, 4 };


	auto ov2 = ov1 * 7.0;
	auto ov3 = 2. * ov1;


	ns_utils::print_container(std::vector<double>(ov1));
	ns_utils::print_container(std::vector<double>(ov2));
	ns_utils::print_container(std::vector<double>(ov3));

	// Test overloaded a*num, b*num
	ns_control_toolbox::tf tf_overloaded{{ 2, 0, 4 },
	                                     { 1, 4, 0 }};

	ns_utils::print("Before vector scalar multiplicaiton");
	tf_overloaded.print();

	tf_overloaded.update_num_coef(10.);
	tf_overloaded.update_den_coef(5.);

	ns_utils::print("After vector scalar multiplicaiton");
	tf_overloaded.print();


	return 0;
}