//
// Created by ali on 21/2/22.
//

#include <fmt/core.h>
#include <cassert>
#include "act_utils.hpp"
#include "autoware_control_toolbox.hpp"


int main()
{

    double Td = 1.1; // time delay in seconds.
    size_t order = 3; // order of the Pade approximation.

    auto tf_delay = ns_control_toolbox::pade(Td, order);
    tf_delay.print();

    auto ss_sys = ns_control_toolbox::tf2ss(tf_delay);

    return 0;
}
