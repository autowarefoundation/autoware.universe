// Copyright 2022 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
