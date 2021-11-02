// Copyright 2019 Autoware Foundation
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

#ifndef MISSION_PLANNER__LANELET2_IMPL__ROUTE_HANDLER_HPP_
#define MISSION_PLANNER__LANELET2_IMPL__ROUTE_HANDLER_HPP_

// lanelet
#include <vector>

// Autoware
#include "mission_planner/lanelet2_impl/utility_functions.hpp"

#include <lanelet2_extension/utility/query.hpp>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>

namespace mission_planner
{
class RouteHandler
{
private:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;

  void setRouteLanelets(
    const lanelet::LaneletMapConstPtr & lanelet_map_ptr,
    const lanelet::routing::RoutingGraphPtr & routing_graph_ptr,
    const lanelet::ConstLanelets & path_lanelets);
  bool isBijectiveConnection(
    const lanelet::ConstLanelets & lanelet_section1,
    const lanelet::ConstLanelets & lanelet_section2) const;

public:
  bool getPreviousLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const;
  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;
  RouteHandler(
    const lanelet::LaneletMapConstPtr & lanelet_map_ptr,
    const lanelet::routing::RoutingGraphPtr & routing_graph,
    const lanelet::ConstLanelets & path_lanelets);
  lanelet::ConstLanelets getRouteLanelets() const;

  lanelet::ConstLanelets getLaneletSequence(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneletSequenceUpTo(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneletSequenceAfter(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getPreviousLaneletSequence(
    const lanelet::ConstLanelets & lanelet_sequence) const;

  lanelet::ConstLanelets getLaneSequenceUpTo(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneSequenceAfter(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneSequence(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getNeighborsWithinRoute(const lanelet::ConstLanelet & lanelet) const;
  std::vector<lanelet::ConstLanelets> getLaneSection(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getNextLaneSequence(const lanelet::ConstLanelets & lane_sequence) const;
};
}  // namespace mission_planner
#endif  // MISSION_PLANNER__LANELET2_IMPL__ROUTE_HANDLER_HPP_
