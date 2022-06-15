// Copyright 2022 TIER IV, Inc.
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

#ifndef ROUTE__TEST_ROUTE_CHECKER_HPP_
#define ROUTE__TEST_ROUTE_CHECKER_HPP_

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::HADMapRoute;

inline Route generateRoute(const HADMapRoute::ConstSharedPtr route)
{
  Route r;
  for (const auto & lanelet : route->lanelets)
  {
    r.lanelets.push_back(lanelet);
  }
  return r
{

} // namespace tier4_autoware_utils

#endif // ROUTE__TEST_ROUTE_CHECKER_HPP_