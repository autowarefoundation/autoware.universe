// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/scene_module/lane_following/lane_following_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "route_handler/lanelet_path.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
LaneFollowingModule::LaneFollowingModule(
  const std::string & name, rclcpp::Node & node, const LaneFollowingParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  initParam();
}

void LaneFollowingModule::initParam()
{
  clearWaitingApproval();  // no need approval
}

bool LaneFollowingModule::isExecutionRequested() const { return true; }

bool LaneFollowingModule::isExecutionReady() const { return true; }

BT::NodeStatus LaneFollowingModule::updateState()
{
  current_state_ = BT::NodeStatus::SUCCESS;
  return current_state_;
}

BehaviorModuleOutput LaneFollowingModule::plan()
{
  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(getReferencePath());
  return output;
}
CandidateOutput LaneFollowingModule::planCandidate() const
{
  return CandidateOutput(getReferencePath());
}
void LaneFollowingModule::onEntry()
{
  initParam();
  current_state_ = BT::NodeStatus::RUNNING;
  RCLCPP_DEBUG(getLogger(), "LANE_FOLLOWING onEntry");
}
void LaneFollowingModule::onExit()
{
  initParam();
  current_state_ = BT::NodeStatus::SUCCESS;
  RCLCPP_DEBUG(getLogger(), "LANE_FOLLOWING onExit");
}

void LaneFollowingModule::setParameters(const LaneFollowingParameters & parameters)
{
  parameters_ = parameters;
}

PathWithLaneId LaneFollowingModule::getReferencePath() const
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto p = planner_data_->parameters;

  const auto lanelet_route_ptr = route_handler->getLaneletRoutePtr();

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  auto current_lanelet_point = lanelet_route_ptr->getClosestLaneletPointWithinRoute(current_pose);
  // auto current_lanelet_path = lanelet_route_ptr->getStraightPath(current_lanelet_point,
  // p.backward_path_length, p.forward_path_length);
  auto current_lanelet_path =
    lanelet_route_ptr->getStraightPathFrom(current_lanelet_point, p.forward_path_length);
  current_lanelet_path = lanelet_route_ptr->extendPath(
    current_lanelet_path, p.backward_path_length, 0., false,
    route_handler::OverlapRemovalStrategy::KEEP_START);

  if (current_lanelet_path.empty()) {
    return reference_path;
  }

  lanelet::ConstLanelets current_lanes =
    behavior_path_planner::util::getPathLanelets(current_lanelet_path);
  const auto drivable_lanes = util::generateDrivableLanes(current_lanes);

  // FIXME(vrichard) I don't understand what we are trying to do here:
  // 1. we generate a path within [-backward, forward]
  // 2. we extend back to [-(backward+extra_margin), forward]
  // 3. we generate centerline path from the extended path (still [-(backward+extra_margin),
  // forward])
  // 4. we clip the centerline to [-backward, forward]
  // What is the point of extending the path backward if we don't care about the extended part ??
  // If it is required to generate a better centerline, then the centerline generation function
  // should take care of this for us.

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 10.0;
  auto current_lanelet_path_with_backward_margin = lanelet_route_ptr->extendPath(
    current_lanelet_path, extra_margin, 0., false,
    route_handler::OverlapRemovalStrategy::KEEP_START);

  if (current_lanelet_path_with_backward_margin.empty()) {
    // FIXME(vrichard) better than nothing?
    current_lanelet_path_with_backward_margin = current_lanelet_path;
  }

  reference_path =
    util::getCenterLinePath(*route_handler, current_lanelet_path_with_backward_margin, p);

  // clip backward length
  const size_t current_seg_idx = findEgoSegmentIndex(reference_path.points);
  util::clipPathLength(
    reference_path, current_seg_idx, p.forward_path_length, p.backward_path_length);

  {
    // NOTE(vrichard) always has value since current_lanelet_point is built within the route.
    const auto optional_num_lane_change =
      lanelet_route_ptr->getNumLaneChangeToPreferredLane(current_lanelet_point);
    const int num_lane_change = std::abs(*optional_num_lane_change);
    double optional_lengths{0.0};
    const auto isInIntersection = util::checkLaneIsInIntersection(
      *route_handler, reference_path, current_lanes, p, num_lane_change, optional_lengths);
    if (isInIntersection) {
      reference_path =
        util::getCenterLinePath(*route_handler, current_lanelet_path, p, optional_lengths);
    }

    const double lane_change_buffer =
      util::calcLaneChangeBuffer(p, num_lane_change, optional_lengths);

    reference_path = util::setDecelerationVelocity(
      *route_handler, reference_path, current_lanelet_path,
      parameters_.lane_change_prepare_duration, lane_change_buffer);
  }

  // const auto shorten_lanes = util::cutOverlappedLanes(reference_path, drivable_lanes);

  const auto expanded_lanes = util::expandLanelets(
    drivable_lanes, parameters_.drivable_area_left_bound_offset,
    parameters_.drivable_area_right_bound_offset);

  util::generateDrivableArea(reference_path, expanded_lanes, p.vehicle_length, planner_data_);

  return reference_path;
}
}  // namespace behavior_path_planner
