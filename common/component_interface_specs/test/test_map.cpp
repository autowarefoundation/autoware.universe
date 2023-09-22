#include "gtest/gtest.h"
#include "component_interface_specs/map.hpp"

TEST(mapinterface,interface)
{
  {
    using map_interface::MapProjectorInfo;
    MapProjectorInfo mapprojectorinfo;    
    size_t depth=1;
    EXPECT_EQ(mapprojectorinfo.depth,depth);
    EXPECT_EQ(mapprojectorinfo.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(mapprojectorinfo.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
