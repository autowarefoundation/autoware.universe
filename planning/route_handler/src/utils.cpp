
#include "route_handler/utils.hpp"

#include <lanelet2_extension/utility/query.hpp>

#include <algorithm>

namespace route_handler::utils
{

bool validatePath(
  const LaneletPath & lanelet_path, const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  // LaneletPath internal validation
  if (!lanelet_path.validate()) {
    return false;
  }

  // checking lanelets connection on the graph
  auto prev_it = lanelet_path.sections().begin();
  auto curr_it = prev_it + 1;
  while (curr_it != lanelet_path.sections().end()) {
    // The previous and current lanelets are connected if a follower of one of the previous lanelet
    // neighbors is a neighbor of the current lanelet
    const lanelet::ConstLanelets prev_neighbors =
      lanelet::utils::query::getAllNeighbors(routing_graph_ptr, prev_it->lanelet());
    const lanelet::ConstLanelets curr_neighbors =
      lanelet::utils::query::getAllNeighbors(routing_graph_ptr, curr_it->lanelet());
    const bool connected =
      std::any_of(prev_neighbors.begin(), prev_neighbors.end(), [&](const auto & prev_llt) {
        const lanelet::ConstLanelets following_lanelets = routing_graph_ptr->following(prev_llt);
        return std::any_of(
          curr_neighbors.begin(), curr_neighbors.end(), [&](const auto & curr_llt) {
            return lanelet::utils::contains(following_lanelets, curr_llt);
          });
      });

    if (!connected) {
      return false;  // not connected
    }

    prev_it = curr_it;
    curr_it++;
  }

  // everything looks ok
  return true;
}

bool isPathStraight(
  const LaneletPath & lanelet_path, const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  if (lanelet_path.empty()) {
    return false;
  }

  // check if all sections are directly connected to each other
  auto prev_it = lanelet_path.begin();
  auto curr_it = prev_it + 1;
  while (curr_it != lanelet_path.end()) {
    const lanelet::ConstLanelets following_lanelets =
      routing_graph_ptr->following(prev_it->lanelet());
    if (!lanelet::utils::contains(following_lanelets, curr_it->lanelet())) {
      return false;  // the next lanelet in the path does not follow current one
    }
  }
  return true;
}

}  // namespace route_handler::utils
