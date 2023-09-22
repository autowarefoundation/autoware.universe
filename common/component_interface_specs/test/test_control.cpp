#include "gtest/gtest.h"
#include "component_interface_specs/control.hpp"

TEST(controlinterface,interface)
{
  {
    using control_interface::IsPaused;
    IsPaused ispaused;    
    size_t depth=1;
    EXPECT_EQ(ispaused.depth,depth);
    EXPECT_EQ(ispaused.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(ispaused.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using control_interface::IsStartRequested;
    IsStartRequested isstartrequested;    
    size_t depth=1;
    EXPECT_EQ(isstartrequested.depth,depth);
    EXPECT_EQ(isstartrequested.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(isstartrequested.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using control_interface::IsStopped;
    IsStopped isstopped;    
    size_t depth=1;
    EXPECT_EQ(isstopped.depth,depth);
    EXPECT_EQ(isstopped.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(isstopped.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
