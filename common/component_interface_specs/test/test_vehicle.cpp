#include "gtest/gtest.h"
#include "component_interface_specs/vehicle.hpp"

TEST(vehicleinterface,interface)
{
  {
    using vehicle_interface::SteeringStatus;
    SteeringStatus steeringstatus;    
    size_t depth=1;
    EXPECT_EQ(steeringstatus.depth,depth);
    EXPECT_EQ(steeringstatus.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(steeringstatus.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::GearStatus;
    GearStatus gearstatus;    
    size_t depth=1;
    EXPECT_EQ(gearstatus.depth,depth);
    EXPECT_EQ(gearstatus.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(gearstatus.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::TurnIndicatorStatus;
    TurnIndicatorStatus turnindicatorstatus;    
    size_t depth=1;
    EXPECT_EQ(turnindicatorstatus.depth,depth);
    EXPECT_EQ(turnindicatorstatus.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(turnindicatorstatus.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::HazardLightStatus;
    HazardLightStatus hazardlightstatus;    
    size_t depth=1;
    EXPECT_EQ(hazardlightstatus.depth,depth);
    EXPECT_EQ(hazardlightstatus.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(hazardlightstatus.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::EnergyStatus;
    EnergyStatus energystatus;    
    size_t depth=1;
    EXPECT_EQ(energystatus.depth,depth);
    EXPECT_EQ(energystatus.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(energystatus.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

}
