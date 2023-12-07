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
#include "behavior_path_planner/utils/drivable_area_expansion/drivable_area_expansion.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/sampling_planner/sampling_planner_parameters.hpp"
#include "behavior_path_planner/utils/sampling_planner/util.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "bezier_sampler/bezier_sampling.hpp"
#include "frenet_planner/frenet_planner.hpp"
#include "lanelet2_extension/utility/query.hpp"
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
#include "tier4_autoware_utils/math/constants.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "tier4_planning_msgs/msg/lateral_offset.hpp"

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
    internal_params_->constraints.soft.weights = user_params_->weights;
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
    const std::vector<geometry_msgs::msg::Point> & right_bound) const;

  PlanResult generatePath();

  bool canTransitSuccessState() override
  {
    std::vector<DrivableLanes> drivable_lanes{};
    const auto & prev_module_path = getPreviousModuleOutput().path;
    const auto prev_module_reference_path = getPreviousModuleOutput().reference_path;

    const auto & p = planner_data_->parameters;
    const auto ego_pose = planner_data_->self_odometry->pose.pose;
    lanelet::ConstLanelet current_lane;

    if (!planner_data_->route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("behavior_path_planner").get_child("utils"),
        "failed to find closest lanelet within route!!!");
      return {};
    }
    const auto current_lane_sequence = planner_data_->route_handler->getLaneletSequence(
      current_lane, ego_pose, p.backward_path_length, p.forward_path_length);
    // expand drivable lanes
    std::for_each(
      current_lane_sequence.begin(), current_lane_sequence.end(), [&](const auto & lanelet) {
        drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
      });

    lanelet::ConstLanelets current_lanes;

    for (auto & d : drivable_lanes) {
      current_lanes.push_back(d.right_lane);
      current_lanes.push_back(d.left_lane);
      current_lanes.insert(current_lanes.end(), d.middle_lanes.begin(), d.middle_lanes.end());
    }

    const auto ego_arc = lanelet::utils::getArcCoordinates(current_lanes, ego_pose);
    const auto goal_pose = planner_data_->route_handler->getGoalPose();
    const auto goal_arc = lanelet::utils::getArcCoordinates(current_lanes, goal_pose);
    const double length_to_goal = std::abs(goal_arc.length - ego_arc.length);
    const double min_target_length = *std::min_element(
      internal_params_->sampling.target_lengths.begin(),
      internal_params_->sampling.target_lengths.end());
    const auto nearest_index =
      motion_utils::findNearestIndex(prev_module_reference_path->points, ego_pose);
    double yaw_difference = 0.0;
    if (!nearest_index) return false;
    auto toYaw = [](const geometry_msgs::msg::Quaternion & quat) -> double {
      geometry_msgs::msg::Vector3 rpy;
      tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
      tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
      return rpy.z;
    };
    const auto quat = prev_module_reference_path->points[*nearest_index].point.pose.orientation;
    const double ref_path_yaw = toYaw(quat);
    const double ego_yaw = toYaw(ego_pose.orientation);
    yaw_difference = std::abs(ego_yaw - ref_path_yaw);
    constexpr double pi = 3.14159;

    lanelet::ConstLanelet closest_lanelet_to_ego;
    lanelet::utils::query::getClosestLanelet(current_lanes, ego_pose, &closest_lanelet_to_ego);
    lanelet::ConstLanelet closest_lanelet_to_goal;
    lanelet::utils::query::getClosestLanelet(current_lanes, goal_pose, &closest_lanelet_to_goal);
    const bool merged_back_to_path =
      ((length_to_goal < min_target_length) || (std::abs(ego_arc.distance)) < 0.5) &&
      (yaw_difference < pi / 36.0) &&
      (closest_lanelet_to_goal.id() == closest_lanelet_to_ego.id());  // TODO(Daniel) magic numbers
    return isReferencePathSafe() && (merged_back_to_path);
  }

  bool canTransitFailureState() override { return false; }

  bool canTransitIdleToRunningState() override { return prev_sampling_path_.has_value(); }

  bool isReferencePathSafe() const;

  void updateDebugMarkers();

  void prepareConstraints(
    sampler_common::Constraints & constraints,
    const PredictedObjects::ConstSharedPtr & predicted_objects,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound) const;

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
    const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane) const;
  DrivableLanes generateExpandDrivableLanes(
    const lanelet::ConstLanelet & lanelet,
    const std::shared_ptr<const PlannerData> & planner_data) const;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SAMPLING_PLANNER__SAMPLING_PLANNER_MODULE_HPP_
