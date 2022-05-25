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


//	std::vector<double> num{ 1 };
//	std::vector<double> den{ 3 };

	// With a num, den
	ns_control_toolbox::tf sys_tf(num, den);

	// Print sys
	sys_tf.print();

	//  Default constructor example
	// ns_control_toolbox::tf2ss sys_ss;
	ns_control_toolbox::tf2ss sys_ss1(sys_tf);

	// Constructor with num and den
	ns_control_toolbox::tf2ss sys_ss2(num, den);
	sys_ss2.print();


	return 0;
}