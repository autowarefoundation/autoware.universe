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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SAMPLING_PLANNER__SAMPLING_PLANNER_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SAMPLING_PLANNER__SAMPLING_PLANNER_MODULE_HPP_

#include "behavior_path_planner/marker_utils/utils.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/sampling_planner/sampling_planner_parameters.hpp"
#include "behavior_path_planner/utils/sampling_planner/util.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "bezier_sampler/bezier_sampling.hpp"
#include "frenet_planner/frenet_planner.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "motion_utils/trajectory/path_with_lane_id.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sampler_common/constraints/footprint.hpp"
#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/constraints/soft_constraint.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <algorithm>
#include <any>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
struct SamplingPlannerData
{
  // input
  std::vector<TrajectoryPoint> traj_points;  // converted from the input path
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;

  // ego
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel{};
};

struct SamplingPlannerDebugData
{
  std::vector<sampler_common::Path> sampled_candidates{};
  size_t previous_sampled_candidates_nb = 0UL;
  std::vector<tier4_autoware_utils::Polygon2d> obstacles{};
  std::vector<tier4_autoware_utils::MultiPoint2d> footprints{};
};
class SamplingPlannerModule : public SceneModuleInterface
{
public:
  SamplingPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<SamplingPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  void updateData() override;

  void updateModuleParams(const std::any & parameters) override
  {
    std::shared_ptr<SamplingPlannerParameters> user_params_ =
      std::any_cast<std::shared_ptr<SamplingPlannerParameters>>(parameters);

    // Constraints
    internal_params_->constraints.hard.max_curvature = user_params_->max_curvature;
    internal_params_->constraints.hard.min_curvature = user_params_->min_curvature;
    internal_params_->constraints.soft.lateral_deviation_weight =
      user_params_->lateral_deviation_weight;
    internal_params_->constraints.soft.length_weight = user_params_->length_weight;
    internal_params_->constraints.soft.curvature_weight = user_params_->curvature_weight;
    internal_params_->constraints.ego_footprint = vehicle_info_.createFootprint(0.75);
    internal_params_->constraints.ego_width = vehicle_info_.vehicle_width_m;
    internal_params_->constraints.ego_length = vehicle_info_.vehicle_length_m;
    // Sampling
    internal_params_->sampling.enable_frenet = user_params_->enable_frenet;
    internal_params_->sampling.enable_bezier = user_params_->enable_bezier;
    internal_params_->sampling.resolution = user_params_->resolution;
    internal_params_->sampling.previous_path_reuse_points_nb =
      user_params_->previous_path_reuse_points_nb;
    internal_params_->sampling.target_lengths = user_params_->target_lengths;
    internal_params_->sampling.target_lateral_positions = user_params_->target_lateral_positions;
    internal_params_->sampling.nb_target_lateral_positions =
      user_params_->nb_target_lateral_positions;

    internal_params_->sampling.frenet.target_lateral_velocities =
      user_params_->target_lateral_velocities;
    internal_params_->sampling.frenet.target_lateral_accelerations =
      user_params_->target_lateral_accelerations;
    // internal_params_->sampling.bezier.nb_k = user_params_->nb_k;
    // internal_params_->sampling.bezier.mk_min = user_params_->mk_min;
    // internal_params_->sampling.bezier.mk_max = user_params_->mk_max;
    // internal_params_->sampling.bezier.nb_t = user_params_->nb_t;
    // internal_params_->sampling.bezier.mt_min = user_params_->mt_min;
    // internal_params_->sampling.bezier.mt_max = user_params_->mt_max;

    // Preprocessing
    internal_params_->preprocessing.force_zero_deviation = user_params_->force_zero_deviation;
    internal_params_->preprocessing.force_zero_heading = user_params_->force_zero_heading;
    internal_params_->preprocessing.smooth_reference = user_params_->smooth_reference;
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  SamplingPlannerDebugData debug_data_;
  behavior_path_planner::HardConstraintsFunctionVector hard_constraints_;
  behavior_path_planner::SoftConstraintsFunctionVector soft_constraints_;

private:
  SamplingPlannerData createPlannerData(
    const PlanResult & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound);

  PlanResult generatePath();

  bool canTransitSuccessState() override { return false; }

  bool canTransitFailureState() override { return false; }

  bool canTransitIdleToRunningState() override { return false; }

  void updateDebugMarkers();

  void prepareConstraints(
    sampler_common::Constraints & constraints,
    const PredictedObjects::ConstSharedPtr & predicted_objects,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound);

  frenet_planner::SamplingParameters prepareSamplingParameters(
    const sampler_common::State & initial_state,
    const sampler_common::transform::Spline2D & path_spline,
    const SamplingPlannerInternalParameters & internal_params_);

  PathWithLaneId convertFrenetPathToPathWithLaneID(
    const frenet_planner::Path frenet_path, const lanelet::ConstLanelets & lanelets,
    const double path_z);

  // member
  // std::shared_ptr<SamplingPlannerParameters> params_;
  std::shared_ptr<SamplingPlannerInternalParameters> internal_params_;
  vehicle_info_util::VehicleInfo vehicle_info_{};
  std::optional<frenet_planner::Path> prev_sampling_path_ = std::nullopt;
  // move to utils

  void extendOutputDrivableArea(BehaviorModuleOutput & output);
  bool isEndPointsConnected(
    const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane);
  DrivableLanes generateExpandDrivableLanes(
    const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data);
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SAMPLING_PLANNER__SAMPLING_PLANNER_MODULE_HPP_
