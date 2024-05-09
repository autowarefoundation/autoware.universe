// Copyright 2024 TIER IV
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

#include "test_route_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace route_handler::test
{
TEST_F(TestRouteHandler, testSomething)
{
  ASSERT_EQ(1, 1);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
}  // namespace route_handler::test
