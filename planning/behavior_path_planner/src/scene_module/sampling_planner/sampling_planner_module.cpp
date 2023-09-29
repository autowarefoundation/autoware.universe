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
  if (getPreviousModuleOutput().reference_path->points.empty()) {
    RCLCPP_WARN(getLogger(), "reference path is empty.");
    return false;
  }

  if (!motion_utils::isDrivingForward(getPreviousModuleOutput().reference_path->points)) {
    RCLCPP_WARN(getLogger(), "Backward path is NOT supported. Just converting path to trajectory");
    return false;
  }
  return true;
}

bool SamplingPlannerModule::isExecutionReady() const
{
  return true;
}

template <typename T>
TrajectoryPoint SamplingPlannerModule::convertToTrajectoryPoint(const T & point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = tier4_autoware_utils::getPose(point);
  traj_point.longitudinal_velocity_mps = 0;
  traj_point.lateral_velocity_mps = 0;
  traj_point.heading_rate_rps = 0;
  return traj_point;
}

template <typename T>
std::vector<TrajectoryPoint> SamplingPlannerModule::convertToTrajectoryPoints(
  const std::vector<T> & points)
{
  std::vector<TrajectoryPoint> traj_points;
  for (const auto & point : points) {
    const auto traj_point = convertToTrajectoryPoint(point);
    traj_points.push_back(traj_point);
  }
  return traj_points;
}

SamplingPlannerData SamplingPlannerModule::createPlannerData(const PlanResult & path)
{
  // create planner data

  SamplingPlannerData data;
  // planner_data.header = path.header;
  auto points = path->points;
  data.traj_points = convertToTrajectoryPoints(points);
  data.left_bound = path->left_bound;
  data.right_bound = path->right_bound;
  data.ego_pose = planner_data_->self_odometry->pose.pose;
  data.ego_vel = 0;
  // data.ego_vel = ego_state_ptr_->twist.twist.linear.x;
  return data;
}

BehaviorModuleOutput SamplingPlannerModule::plan()
{
  // const auto refPath = getPreviousModuleOutput().reference_path;
  const auto path_ptr = getPreviousModuleOutput().reference_path;
  if (path_ptr->points.empty()) {
    return {};
  }
  const auto ego_closest_path_index = planner_data_->findEgoIndex(path_ptr->points);
  RCLCPP_INFO(getLogger(), "ego_closest_path_index %ld", ego_closest_path_index);
  // resetPathCandidate();
  // resetPathReference();
  // [[maybe_unused]] const auto path = toPath(*path_ptr);
  auto reference_spline = [&]() -> sampler_common::transform::Spline2D {
    std::vector<double> x;
    std::vector<double> y;
    x.reserve(path_ptr->points.size());
    y.reserve(path_ptr->points.size());
    for (const auto & point : path_ptr->points) {
      x.push_back(point.point.pose.position.x);
      y.push_back(point.point.pose.position.y);
    }
    return {x, y};
  }();

  const auto pose = planner_data_->self_odometry->pose.pose;
  sampler_common::State initial_state;
  Point2d initial_state_pose{pose.position.x, pose.position.y};
  const auto rpy =
    tier4_autoware_utils::getRPY(planner_data_->self_odometry->pose.pose.orientation);
  initial_state.pose = initial_state_pose;
  initial_state.frenet = reference_spline.frenet({pose.position.x, pose.position.y});
  initial_state.heading = rpy.z;

  frenet_planner::SamplingParameters sampling_parameters =
    prepareSamplingParameters(initial_state, reference_spline, params_);

  frenet_planner::FrenetState frenet_initial_state;
  frenet_initial_state.position = initial_state.frenet;

  auto frenet_paths =
    frenet_planner::generatePaths(reference_spline, frenet_initial_state, sampling_parameters);
  // After path generation we do pruning we then select the optimal path based on obstacles and
  // drivable area and copy to the expected output and finally copy the velocities from the ref path
  RCLCPP_INFO(getLogger(), "Generated paths %ld", frenet_paths.size());

  return {};
}

CandidateOutput SamplingPlannerModule::planCandidate() const
{
  return {};
}

void SamplingPlannerModule::updateData()
{
}

frenet_planner::SamplingParameters SamplingPlannerModule::prepareSamplingParameters(
  const sampler_common::State & initial_state,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  // calculate target lateral positions
  std::vector<double> target_lateral_positions;
  if (params.sampling.nb_target_lateral_positions > 1) {
    target_lateral_positions = {0.0, initial_state.frenet.d};
    double min_d = 0.0;
    double max_d = 0.0;
    double min_d_s = std::numeric_limits<double>::max();
    double max_d_s = std::numeric_limits<double>::max();
    for (const auto & drivable_poly : params.constraints.drivable_polygons) {
      for (const auto & p : drivable_poly.outer()) {
        const auto frenet_coordinates = path_spline.frenet(p);
        const auto d_s = std::abs(frenet_coordinates.s - initial_state.frenet.s);
        if (d_s < min_d_s && frenet_coordinates.d < 0.0) {
          min_d_s = d_s;
          min_d = frenet_coordinates.d;
        }
        if (d_s < max_d_s && frenet_coordinates.d > 0.0) {
          max_d_s = d_s;
          max_d = frenet_coordinates.d;
        }
      }
    }
    min_d += params.constraints.ego_width / 2.0;
    max_d -= params.constraints.ego_width / 2.0;
    if (min_d < max_d) {
      for (auto r = 0.0; r <= 1.0; r += 1.0 / (params.sampling.nb_target_lateral_positions - 1))
        target_lateral_positions.push_back(interpolation::lerp(min_d, max_d, r));
    }
  } else {
    target_lateral_positions = params.sampling.target_lateral_positions;
  }
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.resolution = params.sampling.resolution;
  const auto max_s = path_spline.lastS();
  frenet_planner::SamplingParameter p;
  for (const auto target_length : params.sampling.target_lengths) {
    p.target_state.position.s =
      std::min(max_s, path_spline.frenet(initial_state.pose).s + std::max(0.0, target_length));
    for (const auto target_lateral_position : target_lateral_positions) {
      p.target_state.position.d = target_lateral_position;
      for (const auto target_lat_vel : params.sampling.frenet.target_lateral_velocities) {
        p.target_state.lateral_velocity = target_lat_vel;
        for (const auto target_lat_acc : params.sampling.frenet.target_lateral_accelerations) {
          p.target_state.lateral_acceleration = target_lat_acc;
          sampling_parameters.parameters.push_back(p);
        }
      }
    }
    if (p.target_state.position.s == max_s) break;
  }
  return sampling_parameters;
}

}  // namespace behavior_path_planner
