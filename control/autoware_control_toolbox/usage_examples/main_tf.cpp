//
// Created by ali on 17/2/22.
//

#include <fmt/core.h>
#include <cassert>
#include "utils/act_utils.hpp"
#include "autoware_control_toolbox.hpp"


int main()
{

	std::vector<double> num{ -1, -1, 5 };
	std::vector<double> den{ 1, 5, 1 };

	// With a num, den
	ns_control_toolbox::tf sys(num, den);

	// Print sys
	sys.print();

	// With a default constructor
	ns_utils::print("\n\n");
	ns_control_toolbox::tf sys_default;
	sys_default.print();


	return 0;
}