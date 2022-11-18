#pragma once
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace pcdless ::ll2_decomposer
{
lanelet::LaneletMapPtr from_bin_msg(const autoware_auto_mapping_msgs::msg::HADMapBin & msg);
}