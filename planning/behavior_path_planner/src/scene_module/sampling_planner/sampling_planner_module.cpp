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
using tier4_autoware_utils::Point2d;
using utils::toPath;

SamplingPlannerModule::SamplingPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SamplingPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map}
{
  internal_params_ =
    std::shared_ptr<SamplingPlannerInternalParameters>(new SamplingPlannerInternalParameters{});
  updateModuleParams(parameters);
}

bool SamplingPlannerModule::isExecutionRequested() const
{
  if (getPreviousModuleOutput().reference_path->points.empty()) {
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

SamplingPlannerData SamplingPlannerModule::createPlannerData(const PlanResult & path)
{
  // create planner data
  SamplingPlannerData data;
  // planner_data.header = path.header;
  auto points = path->points;
  data.left_bound = path->left_bound;
  data.right_bound = path->right_bound;
  data.ego_pose = planner_data_->self_odometry->pose.pose;
  data.ego_vel = 0;
  // data.ego_vel = ego_state_ptr_->twist.twist.linear.x;
  return data;
}

PathWithLaneId SamplingPlannerModule::convertFrenetPathToPlanResult(
  const frenet_planner::Path frenet_path, const lanelet::ConstLanelets & lanelets,
  const double velocity)
{
  auto quaternion_from_rpy = [](double roll, double pitch, double yaw) -> tf2::Quaternion {
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    return quaternion_tf2;
  };

  // auto copy_point_information = []()

  PathWithLaneId path;
  const auto header = planner_data_->route_handler->getRouteHeader();

  for (size_t i = 0; i < frenet_path.points.size(); ++i) {
    const auto & frenet_path_point_position = frenet_path.points.at(i);
    const auto & frenet_path_point_yaw = frenet_path.yaws.at(i);
    // const auto & frenet_path_point_velocity = frenet_path.points.;
    PathPointWithLaneId point{};
    point.point.pose.position.x = frenet_path_point_position.x();
    point.point.pose.position.y = frenet_path_point_position.y();
    point.point.pose.position.z = 0.0;

    auto yaw_as_quaternion = quaternion_from_rpy(0.0, 0.0, frenet_path_point_yaw);
    point.point.pose.orientation.w = yaw_as_quaternion.getW();
    point.point.pose.orientation.x = yaw_as_quaternion.getX();
    point.point.pose.orientation.y = yaw_as_quaternion.getY();
    point.point.pose.orientation.z = yaw_as_quaternion.getZ();

    // put the lane that contain waypoints in lane_ids.
    bool is_in_lanes = false;
    for (const auto & lane : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lane)) {
        point.lane_ids.push_back(lane.id());
        is_in_lanes = true;
      }
    }
    // If none of them corresponds, assign the previous lane_ids.
    if (!is_in_lanes && i > 0) {
      point.lane_ids = path.points.at(i - 1).lane_ids;
    }

    point.point.longitudinal_velocity_mps = velocity;
    path.points.push_back(point);
  }
  return path;
}

BehaviorModuleOutput SamplingPlannerModule::plan()
{
  // const auto refPath = getPreviousModuleOutput().reference_path;
  const auto path_ptr = getPreviousModuleOutput().reference_path;
  if (path_ptr->points.empty()) {
    return {};
  }
  // const auto ego_closest_path_index = planner_data_->findEgoIndex(path_ptr->points);
  // RCLCPP_INFO(getLogger(), "ego_closest_path_index %ld", ego_closest_path_index);
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
    prepareSamplingParameters(initial_state, reference_spline, *internal_params_);

  frenet_planner::FrenetState frenet_initial_state;
  frenet_initial_state.position = initial_state.frenet;

  auto frenet_paths =
    frenet_planner::generatePaths(reference_spline, frenet_initial_state, sampling_parameters);
  // After path generation we do pruning we then select the optimal path based on obstacles and
  // drivable area and copy to the expected output and finally copy the velocities from the ref path
  // RCLCPP_INFO(getLogger(), "Generated paths %ld", frenet_paths.size());

  debug_data_.footprints.clear();
  for (auto & path : frenet_paths) {
    const auto footprint =
      sampler_common::constraints::checkHardConstraints(path, internal_params_->constraints);
    debug_data_.footprints.push_back(footprint);
    sampler_common::constraints::calculateCost(
      path, internal_params_->constraints, reference_spline);
  }
  std::vector<sampler_common::Path> candidate_paths;
  const auto move_to_paths = [&candidate_paths](auto & paths_to_move) {
    candidate_paths.insert(
      candidate_paths.end(), std::make_move_iterator(paths_to_move.begin()),
      std::make_move_iterator(paths_to_move.end()));
  };

  move_to_paths(frenet_paths);
  debug_data_.previous_sampled_candidates_nb = debug_data_.sampled_candidates.size();
  debug_data_.sampled_candidates = candidate_paths;
  debug_data_.obstacles = internal_params_->constraints.obstacle_polygons;
  updateDebugMarkers();

  if (frenet_paths.size() < 1) {
    auto p = getPreviousModuleOutput().reference_path;
    BehaviorModuleOutput out;
    out.path = p;
    out.reference_path = p;
    out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
    return out;
  }

  BehaviorModuleOutput out;
  const double velocity = 0.5;
  const double max_length = *std::max_element(
    internal_params_->sampling.target_lengths.begin(),
    internal_params_->sampling.target_lengths.end());
  const auto road_lanes = utils::getExtendedCurrentLanes(planner_data_, 0, max_length, false);
  auto out_path = convertFrenetPathToPlanResult(frenet_paths[0], road_lanes, velocity);
  out.path = std::make_shared<PathWithLaneId>(out_path);
  auto p = getPreviousModuleOutput().reference_path;
  out.reference_path = p;
  out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
  return out;
}

void SamplingPlannerModule::updateDebugMarkers()
{
  debug_marker_.markers.clear();

  const auto header = planner_data_->route_handler->getRouteHeader();
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = header.stamp;
  m.action = m.ADD;
  m.id = 0UL;
  m.type = m.LINE_STRIP;
  m.color.a = 1.0;
  m.scale.x = 0.02;
  m.ns = "candidates";
  for (const auto & c : debug_data_.sampled_candidates) {
    m.points.clear();
    for (const auto & p : c.points)
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    if (c.constraint_results.isValid()) {
      m.color.g = 1.0;
      m.color.r = 0.0;
    } else {
      m.color.r = 1.0;
      m.color.g = 0.0;
    }
    debug_marker_.markers.push_back(m);
    info_marker_.markers.push_back(m);
    ++m.id;
  }
  // m.ns = "footprint";
  // m.id = 0UL;
  // m.type = m.POINTS;
  // m.points.clear();
  // m.color.r = 1.0;
  // m.color.g = 0.0;
  // m.color.b = 1.0;
  // m.scale.y = 0.02;
  // if (!debug_data_.footprints.empty()) {
  //   m.action = m.ADD;
  //   for (const auto & p : debug_data_.footprints[debug_data_.footprints.size()])
  //     m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
  // } else {
  //   m.action = m.DELETE;
  // }
  // m.ns = "debug_path";
  // m.id = 0UL;
  // m.type = m.POINTS;
  // m.points.clear();
  // m.color.g = 1.0;
  // m.color.b = 0.0;
  // m.scale.y = 0.04;
  // if (!debug_data_.sampled_candidates.empty()) {
  //   m.action = m.ADD;
  //   for (const auto & p :
  //        debug_data_.sampled_candidates[debug_data_.sampled_candidates.size()].points)
  //     m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
  // } else {
  //   m.action = m.DELETE;
  // }
  // debug_marker_.markers.push_back(m);
  // info_marker_.markers.push_back(m);
  // m.type = m.LINE_STRIP;
  // m.ns = "obstacles";
  // m.id = 0UL;
  // m.color.r = 1.0;
  // m.color.g = 0.0;
  // m.color.b = 0.0;
  // for (const auto & obs : debug_data_.obstacles) {
  //   m.points.clear();
  //   for (const auto & p : obs.outer())
  //     m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
  //   debug_marker_.markers.push_back(m);
  //   info_marker_.markers.push_back(m);
  //   ++m.id;
  // }
  // m.action = m.DELETE;
  // m.ns = "candidates";
  // for (m.id = debug_data_.sampled_candidates.size();
  //      static_cast<size_t>(m.id) < debug_data_.previous_sampled_candidates_nb; ++m.id) {
  //   debug_marker_.markers.push_back(m);
  //   info_marker_.markers.push_back(m);
  // }
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
  const sampler_common::transform::Spline2D & path_spline,
  const SamplingPlannerInternalParameters & params_)
{
  // calculate target lateral positions
  std::vector<double> target_lateral_positions;
  if (params_.sampling.nb_target_lateral_positions > 1) {
    target_lateral_positions = {0.0, initial_state.frenet.d};
    double min_d = 0.0;
    double max_d = 0.0;
    double min_d_s = std::numeric_limits<double>::max();
    double max_d_s = std::numeric_limits<double>::max();
    for (const auto & drivable_poly : params_.constraints.drivable_polygons) {
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
    min_d += params_.constraints.ego_width / 2.0;
    max_d -= params_.constraints.ego_width / 2.0;
    if (min_d < max_d) {
      for (auto r = 0.0; r <= 1.0; r += 1.0 / (params_.sampling.nb_target_lateral_positions - 1))
        target_lateral_positions.push_back(interpolation::lerp(min_d, max_d, r));
    }
  } else {
    target_lateral_positions = params_.sampling.target_lateral_positions;
  }
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.resolution = params_.sampling.resolution;
  const auto max_s = path_spline.lastS();
  frenet_planner::SamplingParameter p;
  for (const auto target_length : params_.sampling.target_lengths) {
    p.target_state.position.s =
      std::min(max_s, path_spline.frenet(initial_state.pose).s + std::max(0.0, target_length));
    for (const auto target_lateral_position : target_lateral_positions) {
      p.target_state.position.d = target_lateral_position;
      for (const auto target_lat_vel : params_.sampling.frenet.target_lateral_velocities) {
        p.target_state.lateral_velocity = target_lat_vel;
        for (const auto target_lat_acc : params_.sampling.frenet.target_lateral_accelerations) {
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
