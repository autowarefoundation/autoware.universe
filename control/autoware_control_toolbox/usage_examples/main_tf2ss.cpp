//
// Created by ali on 17/2/22.
//

#include <fmt/core.h>
#include <cassert>
#include "act_utils.hpp"
#include "autoware_control_toolbox.hpp"
#include "utils/writetopath.h"


int main()
{

    auto log_path = getOutputPath() / "tf2ss";

    std::vector<double> num{1, 0.1, 4};
    std::vector<double> den{5, 0.15, 4, 2, 0.365};


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


    // Save as text
    writeToFile(log_path, sys_ss1.A_, "A");

    ns_utils::print(fs::current_path());


    return 0;
}