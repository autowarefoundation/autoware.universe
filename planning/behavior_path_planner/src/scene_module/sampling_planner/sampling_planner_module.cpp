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
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
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

SamplingPlannerData SamplingPlannerModule::createPlannerData(
  const PlanResult & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  // create planner data
  SamplingPlannerData data;
  // planner_data.header = path.header;
  auto points = path->points;
  data.left_bound = left_bound;
  data.right_bound = right_bound;
  data.ego_pose = planner_data_->self_odometry->pose.pose;
  data.ego_vel = planner_data_->self_odometry->twist.twist.linear.x;
  // data.ego_vel = ego_state_ptr_->twist.twist.linear.x;
  return data;
}

PathWithLaneId SamplingPlannerModule::convertFrenetPathToPathWithLaneID(
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
      const auto closest_point =
        lanelet::utils::getClosestCenterPose(lane, point.point.pose.position);
      point.point.pose.position.z = closest_point.position.z;

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

void SamplingPlannerModule::prepareConstraints(
  sampler_common::Constraints & constraints,
  const PredictedObjects::ConstSharedPtr & predicted_objects,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  constraints.obstacle_polygons = sampler_common::MultiPolygon2d();
  for (const auto & o : predicted_objects->objects)
    if (o.kinematics.initial_twist_with_covariance.twist.linear.x < 0.5)  // TODO(Maxime): param
      constraints.obstacle_polygons.push_back(tier4_autoware_utils::toPolygon2d(o));

  constraints.dynamic_obstacles = {};  // TODO(Maxime): not implemented

  // TODO(Maxime): directly use lines instead of polygon

  sampler_common::Polygon2d drivable_area_polygon;
  for (const auto & p : right_bound) {
    drivable_area_polygon.outer().emplace_back(p.x, p.y);
  }
  for (auto it = left_bound.rbegin(); it != left_bound.rend(); ++it)
    drivable_area_polygon.outer().emplace_back(it->x, it->y);

  if (drivable_area_polygon.outer().size() < 1) {
    return;
  }
  drivable_area_polygon.outer().push_back(drivable_area_polygon.outer().front());

  constraints.drivable_polygons = {drivable_area_polygon};
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
  lanelet::ConstLanelet closest_lanelet;
  planner_data_->route_handler->getClosestLaneletWithinRoute(pose, &closest_lanelet);

  sampler_common::State initial_state;
  Point2d initial_state_pose{pose.position.x, pose.position.y};
  const auto rpy =
    tier4_autoware_utils::getRPY(planner_data_->self_odometry->pose.pose.orientation);
  initial_state.pose = initial_state_pose;
  initial_state.frenet = reference_spline.frenet({pose.position.x, pose.position.y});
  initial_state.heading = rpy.z;

  frenet_planner::SamplingParameters sampling_parameters =
    prepareSamplingParameters(initial_state, reference_spline, *internal_params_);

  // const auto drivable_lanes = getPreviousModuleOutput().drivable_area_info.drivable_lanes;

  const auto prev_module_path = getPreviousModuleOutput().path;
  const auto current_lanes = utils::getCurrentLanesFromPath(*prev_module_path, planner_data_);

  std::vector<DrivableLanes> drivable_lanes{};
  // expand drivable lanes
  std::for_each(current_lanes.begin(), current_lanes.end(), [&](const auto & lanelet) {
    drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
  });

  const auto left_bound =
    (utils::calcBound(planner_data_->route_handler, drivable_lanes, false, false, true));
  const auto right_bound =
    (utils::calcBound(planner_data_->route_handler, drivable_lanes, false, false, false));

  const auto sampling_planner_data =
    createPlannerData(planner_data_->prev_output_path, left_bound, right_bound);

  prepareConstraints(
    internal_params_->constraints, planner_data_->dynamic_object, sampling_planner_data.left_bound,
    sampling_planner_data.right_bound);

  frenet_planner::FrenetState frenet_initial_state;
  frenet_initial_state.position = initial_state.frenet;

  auto frenet_paths =
    frenet_planner::generatePaths(reference_spline, frenet_initial_state, sampling_parameters);

  debug_data_.footprints.clear();

  for (auto & path : frenet_paths) {
    const auto footprint =
      sampler_common::constraints::checkHardConstraints(path, internal_params_->constraints);
    path.constraint_results.curvature = true;
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

  const auto best_path_idx = [](const auto & paths) {
    auto min_cost = std::numeric_limits<double>::max();
    size_t best_path_idx = 0;
    for (auto i = 0LU; i < paths.size(); ++i) {
      if (paths[i].constraint_results.isValid() && paths[i].cost < min_cost) {
        best_path_idx = i;
        min_cost = paths[i].cost;
      }
    }
    return min_cost < std::numeric_limits<double>::max() ? std::optional<size_t>(best_path_idx)
                                                         : std::nullopt;
  };
  const auto selected_path_idx = best_path_idx(frenet_paths);

  if (!selected_path_idx) {
    BehaviorModuleOutput out;
    const auto p = getPreviousModuleOutput().reference_path;
    out.path = p;
    out.reference_path = p;
    out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
    return out;
  }

  BehaviorModuleOutput out;
  const double velocity = 1.5;
  const double max_length = *std::max_element(
    internal_params_->sampling.target_lengths.begin(),
    internal_params_->sampling.target_lengths.end());
  const auto road_lanes = utils::getExtendedCurrentLanes(planner_data_, 0, max_length, false);

  const auto best_path = frenet_paths[*selected_path_idx];
  const auto out_path = convertFrenetPathToPathWithLaneID(best_path, road_lanes, velocity);
  out.path = std::make_shared<PathWithLaneId>(out_path);
  const auto p = getPreviousModuleOutput().reference_path;
  out.reference_path = p;
  out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
  extendOutputDrivableArea(out);
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
  m.ns = "footprint";
  m.id = 0UL;
  m.type = m.POINTS;
  m.points.clear();
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 1.0;
  m.scale.y = 0.2;
  if (!debug_data_.footprints.empty()) {
    m.action = m.ADD;
    for (const auto & p : debug_data_.footprints[0]) {
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    }

  } else {
    m.action = m.DELETE;
  }
  debug_marker_.markers.push_back(m);
  info_marker_.markers.push_back(m);
  ++m.id;
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

// lanelet::ConstLanelets NormalLaneChange::getCurrentLanes() const
// {
//   return utils::getCurrentLanesFromPath(prev_module_path_, planner_data_);
// }

void SamplingPlannerModule::extendOutputDrivableArea(BehaviorModuleOutput & output)
{
  const auto prev_module_path = getPreviousModuleOutput().path;
  const auto current_lanes = utils::getCurrentLanesFromPath(*prev_module_path, planner_data_);

  std::vector<DrivableLanes> drivable_lanes{};
  // expand drivable lanes
  std::for_each(current_lanes.begin(), current_lanes.end(), [&](const auto & lanelet) {
    drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
  });

  // // const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
  // //   *planner_data_->route_handler, current_lanes, status_.target_lanes);
  // const auto drivable_lanes = behavior_path_planner::utils::generateDrivableLanes(current_lanes);
  // const auto shorten_lanes = utils::cutOverlappedLanes(*output.path, drivable_lanes);
  // const auto expanded_lanes = utils::expandLanelets(
  //   shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
  //   dp.drivable_area_types_to_skip);

  // // for new architecture
  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = drivable_lanes;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
}

CandidateOutput SamplingPlannerModule::planCandidate() const
{
  return {};
}

void SamplingPlannerModule::updateData()
{
}

// utils

template <typename T>
void pushUniqueVector(T & base_vector, const T & additional_vector)
{
  base_vector.insert(base_vector.end(), additional_vector.begin(), additional_vector.end());
}

bool SamplingPlannerModule::isEndPointsConnected(
  const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane)
{
  const auto & left_back_point_2d = right_lane.leftBound2d().back().basicPoint();
  const auto & right_back_point_2d = left_lane.rightBound2d().back().basicPoint();

  constexpr double epsilon = 1e-5;
  return (right_back_point_2d - left_back_point_2d).norm() < epsilon;
}

DrivableLanes SamplingPlannerModule::generateExpandDrivableLanes(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & route_handler = planner_data->route_handler;

  DrivableLanes current_drivable_lanes;
  current_drivable_lanes.left_lane = lanelet;
  current_drivable_lanes.right_lane = lanelet;

  // 1. get left/right side lanes
  const auto update_left_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_left_lanelets =
      route_handler->getAllLeftSharedLinestringLanelets(target_lane, true, true);
    if (!all_left_lanelets.empty()) {
      current_drivable_lanes.left_lane = all_left_lanelets.back();  // leftmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_left_lanelets.begin(), all_left_lanelets.end() - 1));
    }
  };
  const auto update_right_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_right_lanelets =
      route_handler->getAllRightSharedLinestringLanelets(target_lane, true, true);
    if (!all_right_lanelets.empty()) {
      current_drivable_lanes.right_lane = all_right_lanelets.back();  // rightmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_right_lanelets.begin(), all_right_lanelets.end() - 1));
    }
  };

  update_left_lanelets(lanelet);
  update_right_lanelets(lanelet);

  // 2.1 when there are multiple lanes whose previous lanelet is the same
  const auto get_next_lanes_from_same_previous_lane =
    [&route_handler](const lanelet::ConstLanelet & lane) {
      // get previous lane, and return false if previous lane does not exist
      lanelet::ConstLanelets prev_lanes;
      if (!route_handler->getPreviousLaneletsWithinRoute(lane, &prev_lanes)) {
        return lanelet::ConstLanelets{};
      }

      lanelet::ConstLanelets next_lanes;
      for (const auto & prev_lane : prev_lanes) {
        const auto next_lanes_from_prev = route_handler->getNextLanelets(prev_lane);
        pushUniqueVector(next_lanes, next_lanes_from_prev);
      }
      return next_lanes;
    };

  const auto next_lanes_for_right =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.right_lane);
  const auto next_lanes_for_left =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.left_lane);

  // 2.2 look for neighbor lane recursively, where end line of the lane is connected to end line
  // of the original lane
  const auto update_drivable_lanes =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      for (const auto & next_lane : next_lanes) {
        const auto & edge_lane =
          is_left ? current_drivable_lanes.left_lane : current_drivable_lanes.right_lane;
        if (next_lane.id() == edge_lane.id()) {
          continue;
        }

        const auto & left_lane = is_left ? next_lane : edge_lane;
        const auto & right_lane = is_left ? edge_lane : next_lane;
        if (!isEndPointsConnected(left_lane, right_lane)) {
          continue;
        }

        if (is_left) {
          current_drivable_lanes.left_lane = next_lane;
        } else {
          current_drivable_lanes.right_lane = next_lane;
        }

        const auto & middle_lanes = current_drivable_lanes.middle_lanes;
        const auto has_same_lane = std::any_of(
          middle_lanes.begin(), middle_lanes.end(),
          [&edge_lane](const auto & lane) { return lane.id() == edge_lane.id(); });

        if (!has_same_lane) {
          if (is_left) {
            if (current_drivable_lanes.right_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          } else {
            if (current_drivable_lanes.left_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          }
        }

        return true;
      }
      return false;
    };

  const auto expand_drivable_area_recursively =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      // NOTE: set max search num to avoid infinity loop for drivable area expansion
      constexpr size_t max_recursive_search_num = 3;
      for (size_t i = 0; i < max_recursive_search_num; ++i) {
        const bool is_update_kept = update_drivable_lanes(next_lanes, is_left);
        if (!is_update_kept) {
          break;
        }
        if (i == max_recursive_search_num - 1) {
          RCLCPP_ERROR(
            rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
            "Drivable area expansion reaches max iteration.");
        }
      }
    };
  expand_drivable_area_recursively(next_lanes_for_right, false);
  expand_drivable_area_recursively(next_lanes_for_left, true);

  // 3. update again for new left/right lanes
  update_left_lanelets(current_drivable_lanes.left_lane);
  update_right_lanelets(current_drivable_lanes.right_lane);

  // 4. compensate that current_lane is in either of left_lane, right_lane or middle_lanes.
  if (
    current_drivable_lanes.left_lane.id() != lanelet.id() &&
    current_drivable_lanes.right_lane.id() != lanelet.id()) {
    current_drivable_lanes.middle_lanes.push_back(lanelet);
  }

  return current_drivable_lanes;
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
