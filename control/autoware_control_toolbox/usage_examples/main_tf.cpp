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

	// Transfer function multiplying num or den by a constant.
	std::vector<double> ss{ 1., 2., 3 };
	ns_utils::print_container(ss);

	for (auto& s: ss)
	{
		s *= 5;
		ns_utils::print("s: ", s);

	}

	ns_utils::print_container(ss);


	//
	double x2(1.);
	double y2(1.5);
	auto   anw = ns_utils::isEqual(x2, y2);

	int  x1(1);
	int  y1(1);
	auto anw1 = ns_utils::isEqual(x1, y1);

	ns_control_toolbox::std_vector_overloaded<double> ov1{ 1, 2, 4 };

	//
	auto ov2 = ov1 * 6;


	ns_utils::print_container(std::vector<double>(ov1));
	ns_utils::print_container(std::vector<double>(ov2));

	ov2 *= 7;
	ns_utils::print_container(std::vector<double>(ov2));


	return 0;
}