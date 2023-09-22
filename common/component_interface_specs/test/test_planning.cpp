#include "gtest/gtest.h"
#include "component_interface_specs/planning.hpp"

TEST(planninginterface,interface)
{
    {
        using planning_interface::RouteState;
        RouteState routstate;    
        size_t depth=1;
        EXPECT_EQ(routstate.depth,depth);
        EXPECT_EQ(routstate.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(routstate.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    }

    {
        using planning_interface::Route;
        Route route;    
        size_t depth=1;
        EXPECT_EQ(route.depth,depth);
        EXPECT_EQ(route.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(route.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    }

    {
        using planning_interface::NormalRoute;
        NormalRoute normalroute;    
        size_t depth=1;
        EXPECT_EQ(normalroute.depth,depth);
        EXPECT_EQ(normalroute.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(normalroute.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    }

    {
        using planning_interface::MrmRoute;
        MrmRoute mrmroute;    
        size_t depth=1;
        EXPECT_EQ(mrmroute.depth,depth);
        EXPECT_EQ(mrmroute.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(mrmroute.durability,RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    }

    {
        using planning_interface::Trajectory;
        Trajectory trajectory;    
        size_t depth=1;
        EXPECT_EQ(trajectory.depth,depth);
        EXPECT_EQ(trajectory.reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(trajectory.durability,RMW_QOS_POLICY_DURABILITY_VOLATILE);
    }

}
