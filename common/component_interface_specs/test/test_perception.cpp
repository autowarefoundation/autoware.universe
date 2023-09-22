#include "gtest/gtest.h"
#include "component_interface_specs/perception.hpp"

TEST(perceptioninterface,interface)
{
  {
    using perception_interface::ObjectRecognition;
    ObjectRecognition objectrecognition;    
    size_t depth=1;
    EXPECT_EQ(objectrecognition.depth,depth);
    EXPECT_EQ(objectrecognition.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(objectrecognition.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
}
