#include "gtest/gtest.h"
#include "component_interface_specs/control.hpp"

TEST(controlinterface,interface)
{
  {
    using control_interface::IsPaused;
    IsPaused is_paused;    
    size_t depth=1;
    EXPECT_EQ(is_paused.depth,depth);
    EXPECT_EQ(is_paused.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_paused.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using control_interface::IsStartRequested;
    IsStartRequested is_start_requested;    
    size_t depth=1;
    EXPECT_EQ(is_start_requested.depth,depth);
    EXPECT_EQ(is_start_requested.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_start_requested.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using control_interface::IsStopped;
    IsStopped is_stopped;    
    size_t depth=1;
    EXPECT_EQ(is_stopped.depth,depth);
    EXPECT_EQ(is_stopped.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_stopped.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
