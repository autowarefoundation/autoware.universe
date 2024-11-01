// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_GENERATOR__PATH_HANDLER_HPP_
#define AUTOWARE__PATH_GENERATOR__PATH_HANDLER_HPP_

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <path_generator_parameters.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_routing/RoutingGraph.h>

namespace autoware::path_generator
{
using geometry_msgs::msg::Pose;
using ::path_generator::Params;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

class PathHandler
{
public:
  PathHandler() = default;

  explicit PathHandler(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
  : vehicle_info_(vehicle_info)
  {
  }

  PathWithLaneId generateCenterLinePath(const Pose & current_pose, const Params & param);

  PathWithLaneId generateCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const Pose & current_pose,
    const Params & param);

  PathHandler & setRoute(
    const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & lanelet_map_bin_ptr,
    const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr & route_ptr);

  PathHandler & setVehicleInfo(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
  {
    vehicle_info_ = vehicle_info;
    return *this;
  }

private:
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_{nullptr};
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_{nullptr};

  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;

  std::optional<autoware::vehicle_info_utils::VehicleInfo> vehicle_info_{std::nullopt};

  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end);

  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
    const double backward_path_length, const double forward_path_length);

  lanelet::ConstLanelets getLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet, const double min_length);

  lanelet::ConstLanelets getLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet, const double min_length);

  bool getNextLaneletsWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & next_lanelets);

  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet & next_lanelet);

  bool getPreviousLaneletsWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & prev_lanelets);
};
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__PATH_HANDLER_HPP_
