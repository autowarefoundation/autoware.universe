#include "gtest/gtest.h"
#include "component_interface_specs/localization.hpp"

TEST(localizationinterface,interface)
{
  {
    using localization_interface::InitializationState;
    InitializationState initializationstate;    
    size_t depth=1;
    EXPECT_EQ(initializationstate.depth,depth);
    EXPECT_EQ(initializationstate.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(initializationstate.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using localization_interface::KinematicState;
    KinematicState kinematicstate;    
    size_t depth=1;
    EXPECT_EQ(kinematicstate.depth,depth);
    EXPECT_EQ(kinematicstate.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(kinematicstate.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using localization_interface::Acceleration;
    Acceleration acceleration;    
    size_t depth=1;
    EXPECT_EQ(acceleration.depth,depth);
    EXPECT_EQ(acceleration.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(acceleration.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
}
