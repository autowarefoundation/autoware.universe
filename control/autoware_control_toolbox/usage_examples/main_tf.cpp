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
		
		
		return 0;
	}