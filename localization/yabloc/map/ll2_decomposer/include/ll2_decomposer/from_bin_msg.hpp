#pragma once
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace map
{
lanelet::LaneletMapPtr fromBinMsg(const autoware_auto_mapping_msgs::msg::HADMapBin & msg);
}