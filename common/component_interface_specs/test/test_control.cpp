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
