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

#include "route_parser.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace route_handler::test
{
TEST_F(TestRouteHandler, isRouteHandlerReadyTest)
{
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("route_handler");
  const auto rh_test_route = planning_test_utils_dir + "/test_route/rh_test.route";
  ASSERT_FALSE(route_handler_->isHandlerReady());
  route_handler_->setRoute(parse_route_file(rh_test_route));
  ASSERT_TRUE(route_handler_->isHandlerReady());
}

TEST_F(TestRouteHandler, checkIfIDReturned)
{
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("route_handler");
  const auto rh_test_route = planning_test_utils_dir + "/test_route/rh_test.route";

  route_handler_->setRoute(parse_route_file(rh_test_route));
  const auto lanelet = route_handler_->getLaneletsFromId(4870);

  const auto is_in_goal_route_section = route_handler_->isInGoalRouteSection(lanelet);

  ASSERT_TRUE(is_in_goal_route_section);
}
}  // namespace route_handler::test
