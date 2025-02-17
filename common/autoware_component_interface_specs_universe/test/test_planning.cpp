// Copyright 2023 The Autoware Contributors
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

#include "autoware/component_interface_specs_universe/planning.hpp"
#include "gtest/gtest.h"

TEST(planning, interface)
{
  {
    using autoware::component_interface_specs_universe::planning::RouteState;
    RouteState state;
    size_t depth = 1;
    EXPECT_EQ(state.depth, depth);
    EXPECT_EQ(state.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(state.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using autoware::component_interface_specs_universe::planning::LaneletRoute;
    LaneletRoute route;
    size_t depth = 1;
    EXPECT_EQ(route.depth, depth);
    EXPECT_EQ(route.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(route.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using autoware::component_interface_specs_universe::planning::Trajectory;
    Trajectory trajectory;
    size_t depth = 1;
    EXPECT_EQ(trajectory.depth, depth);
    EXPECT_EQ(trajectory.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(trajectory.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
}
