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

#include "behavior_path_planner/scene_module/sampling_planner/sampling_planner_module.hpp"

namespace behavior_path_planner
{
using geometry_msgs::msg::Point;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::getPoint;
using utils::toPath;

SamplingPlannerModule::SamplingPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SamplingPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map}, parameters_{parameters}
{
  // parameters
  params_.constraints.hard.max_curvature = 0.1;
  params_.constraints.hard.min_curvature = -0.1;
  params_.constraints.soft.lateral_deviation_weight = 1.0;
  params_.constraints.soft.length_weight = 1.0;
  params_.constraints.soft.curvature_weight = 1.0;
  params_.sampling.enable_frenet = true;
  params_.sampling.enable_bezier = false;
  params_.sampling.resolution = 0.5;
  params_.sampling.previous_path_reuse_points_nb = 2;
  params_.sampling.target_lengths = {10.0, 20.0};
  params_.sampling.target_lateral_positions = {-2.0, -1.0, 0.0, 1.0, 2.0};
  params_.sampling.nb_target_lateral_positions = 2;
  params_.sampling.frenet.target_lateral_velocities = {-0.1, 0.0, 0.1};
  params_.sampling.frenet.target_lateral_accelerations = {0.0};
  // params_.sampling.bezier.nb_k = declare_parameter<int>("sampling.bezier.nb_k");
  // params_.sampling.bezier.mk_min = declare_parameter<double>("sampling.bezier.mk_min");
  // params_.sampling.bezier.mk_max = declare_parameter<double>("sampling.bezier.mk_max");
  // params_.sampling.bezier.nb_t = declare_parameter<int>("sampling.bezier.nb_t");
  // params_.sampling.bezier.mt_min = declare_parameter<double>("sampling.bezier.mt_min");
  // params_.sampling.bezier.mt_max = declare_parameter<double>("sampling.bezier.mt_max");
  params_.preprocessing.force_zero_deviation = true;
  params_.preprocessing.force_zero_heading = true;
  params_.preprocessing.smooth_reference = false;
  params_.constraints.ego_footprint = vehicle_info_.createFootprint();
  params_.constraints.ego_width = vehicle_info_.vehicle_width_m;
  params_.constraints.ego_length = vehicle_info_.vehicle_length_m;
}

bool SamplingPlannerModule::isExecutionRequested() const
{
  if (planner_data_->reference_path->points.empty()) {
    RCLCPP_WARN(getLogger(), "reference path is empty.");
    return false;
  }
  return true;
}

bool SamplingPlannerModule::isExecutionReady() const
{
  return true;
}

BehaviorModuleOutput SamplingPlannerModule::plan()
{
  const auto reference_path = planner_data_->reference_path;
  if (reference_path->points.empty()) {
    RCLCPP_WARN(getLogger(), "reference path is empty.");
    return {};
  }
  const auto ego_closest_path_index = planner_data_->findEgoIndex(reference_path->points);
  RCLCPP_INFO(getLogger(), "ego_closest_path_index %ld", ego_closest_path_index);
  // resetPathCandidate();
  // resetPathReference();
  // const auto path = toPath(*reference_path);
  // size_t end_idx = utils::getIdxByArclength(
  //   *reference_path, ego_closest_path_index, params_.sampling.target_lengths[0]);

  auto clipped_path = *reference_path;

  utils::clipPathLength(
    clipped_path, ego_closest_path_index, params_.sampling.target_lengths[0], 0);

  BehaviorModuleOutput output;

  output.reference_path = getPreviousModuleOutput().reference_path;
  // output.reference_path = reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto reference_pose = planner_data_->self_odometry->pose.pose;
  const auto & p = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(reference_pose, &current_lane)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 5000, "failed to find closest lanelet within route!!!");
  }

  // For current_lanes with desired length
  auto current_lanelets_ = route_handler->getLaneletSequence(
    current_lane, reference_pose, p.backward_path_length, p.forward_path_length);

  const auto & dp = planner_data_->drivable_area_expansion_parameters;
  const auto drivable_lanes = utils::generateDrivableLanes(current_lanelets_);
  const auto shorten_lanes = utils::cutOverlappedLanes(clipped_path, drivable_lanes);
  const auto expanded_lanes =
    utils::expandLanelets(shorten_lanes, 2.0, 2.0, dp.drivable_area_types_to_skip);

  output.drivable_area_info.drivable_lanes = expanded_lanes;
  output.drivable_area_info.is_already_expanded = true;
  output.path = std::make_shared<PathWithLaneId>(clipped_path);

  return output;
}

CandidateOutput SamplingPlannerModule::planCandidate() const
{
  return {};
}

void SamplingPlannerModule::updateData()
{
}

}  // namespace behavior_path_planner
