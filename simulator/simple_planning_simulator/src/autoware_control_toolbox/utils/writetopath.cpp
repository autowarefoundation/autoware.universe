//
// Created by ali on 14/8/19.
//

#include "autoware_control_toolbox/utils/writetopath.hpp"

fs::path getOutputPath()
{
    return fs::path("..") / "analyze_outputs/" / "logs";
}
