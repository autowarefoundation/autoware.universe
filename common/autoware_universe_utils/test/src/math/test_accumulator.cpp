// Copyright 2024 Tier IV, Inc.
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

#include "autoware/universe_utils/math/accumulator.hpp"

#include <gtest/gtest.h>

TEST(accumulator, statistic)
{
  autoware::universe_utils::Accumulator<double> acc;
  acc.add(1.0);
  acc.add(2.0);
  acc.add(3.0);

  EXPECT_DOUBLE_EQ(acc.mean(), 2.0);
  EXPECT_DOUBLE_EQ(acc.min(), 1.0);
  EXPECT_DOUBLE_EQ(acc.max(), 3.0);
}
