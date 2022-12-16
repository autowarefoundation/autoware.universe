// Copyright 2022 Macnica, Inc.
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

#ifndef ROUTE_HANDLER__UTILS_HPP_
#define ROUTE_HANDLER__UTILS_HPP_

#include "route_handler/lanelet_path.hpp"

#include <lanelet2_routing/RoutingGraph.h>

namespace route_handler::utils
{

//! @brief Whether the path is valid (well-formed, no loop, no gap, etc.)
bool validatePath(
  const LaneletPath & lanelet_path, 
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr);

//! @brief Whether the path is straight (contains no lane change)
bool isPathStraight(
  const LaneletPath & lanelet_path,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr);

}  // namespace route_handler::utils

#endif  // ROUTE_HANDLER__UTILS_HPP_
