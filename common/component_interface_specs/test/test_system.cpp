#include "gtest/gtest.h"
#include "component_interface_specs/system.hpp"

TEST(systeminterface,interface)
{
  {
    using system_interface::MrmState;
    MrmState mrmstate;    
    size_t depth=1;
    EXPECT_EQ(mrmstate.depth,depth);
    EXPECT_EQ(mrmstate.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(mrmstate.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using system_interface::OperationModeState;
    OperationModeState operationmodeState;    
    size_t depth=1;
    EXPECT_EQ(operationmodeState.depth,depth);
    EXPECT_EQ(operationmodeState.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(operationmodeState.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
