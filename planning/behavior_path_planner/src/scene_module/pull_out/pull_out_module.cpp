// Copyright 2022 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_out/util.hpp"
#include "behavior_path_planner/util/create_vehicle_footprint.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using motion_utils::calcLongitudinalOffsetPose;
using tier4_autoware_utils::calcOffsetPose;
namespace behavior_path_planner
{
PullOutModule::PullOutModule(
  const std::string & name, rclcpp::Node & node, const PullOutParameters & parameters)
: SceneModuleInterface{name, node},
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  rtc_interface_ptr_ = std::make_shared<RTCInterface>(&node, "pull_out");
  lane_departure_checker_ = std::make_shared<LaneDepartureChecker>();
  lane_departure_checker_->setVehicleInfo(vehicle_info_);

  // set enabled planner
  if (parameters_.enable_shift_pull_out) {
    pull_out_planners_.push_back(
      std::make_shared<ShiftPullOut>(node, parameters, lane_departure_checker_));
  }
  if (parameters_.enable_geometric_pull_out) {
    pull_out_planners_.push_back(
      std::make_shared<GeometricPullOut>(node, parameters, getGeometricPullOutParameters()));
  }
  if (pull_out_planners_.empty()) {
    RCLCPP_DEBUG(getLogger(), "Not found enabled planner");
  }

  // debug publisher
  pull_out_start_pose_pub_ = node.create_publisher<PoseStamped>("~/pull_out/debug/backed_pose", 1);
  full_path_pose_array_pub_ =
    node.create_publisher<PoseArray>("~/pull_out/debug/full_path_pose_array", 1);
}

BehaviorModuleOutput PullOutModule::run()
{
  clearWaitingApproval();
  current_state_ = BT::NodeStatus::RUNNING;
  return plan();
}

void PullOutModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OUT onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;

  // initialize when receiving new route
  if (
    last_route_received_time_ == nullptr ||
    *last_route_received_time_ != planner_data_->route_handler->getRouteHeader().stamp) {
    RCLCPP_INFO(getLogger(), "Receive new route, so reset status");
    resetStatus();
    updatePullOutStatus();
  }
  last_route_received_time_ =
    std::make_unique<rclcpp::Time>(planner_data_->route_handler->getRouteHeader().stamp);

  // for preventing chattering between back and pull_out
  if (!status_.back_finished) {
    if (last_pull_out_start_update_time_ == nullptr) {
      last_pull_out_start_update_time_ = std::make_unique<rclcpp::Time>(clock_->now());
    }
    const auto elpased_time = (clock_->now() - *last_pull_out_start_update_time_).seconds();
    if (elpased_time < parameters_.backward_path_update_duration) {
      return;
    }
    last_pull_out_start_update_time_ = std::make_unique<rclcpp::Time>(clock_->now());
  }

  updatePullOutStatus();
}

void PullOutModule::onExit()
{
  clearWaitingApproval();
  removeRTCStatus();
  current_state_ = BT::NodeStatus::IDLE;
  RCLCPP_DEBUG(getLogger(), "PULL_OUT onExit");
}

bool PullOutModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  const bool is_stopped = util::l2Norm(planner_data_->self_odometry->twist.twist.linear) <
                          parameters_.th_arrived_distance;

  lanelet::Lanelet closest_shoulder_lanelet;
  if (
    lanelet::utils::query::getClosestLanelet(
      planner_data_->route_handler->getShoulderLanelets(), planner_data_->self_pose->pose,
      &closest_shoulder_lanelet) &&
    is_stopped) {
    // Create vehicle footprint
    const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info_);
    const auto vehicle_footprint = transformVector(
      local_vehicle_footprint,
      tier4_autoware_utils::pose2transform(planner_data_->self_pose->pose));
    const auto road_lanes = getCurrentLanes();

    // check if goal pose is in shoulder lane and distance is long enough for pull out
    if (isInLane(closest_shoulder_lanelet, vehicle_footprint)) {
      return true;
    }
  }

  return false;
}

bool PullOutModule::isExecutionReady() const { return true; }

// this runs only when RUNNING
BT::NodeStatus PullOutModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OUT updateState");

  if (hasFinishedPullOut()) {
    current_state_ = BT::NodeStatus::SUCCESS;
    return current_state_;
  }

  checkBackFinished();

  return current_state_;
}

BehaviorModuleOutput PullOutModule::plan()
{
  BehaviorModuleOutput output;
  if (!status_.is_safe) {
    return output;
  }

  PathWithLaneId path;
  if (status_.back_finished) {
    if (hasFinishedCurrentPath()) {
      RCLCPP_INFO(getLogger(), "Increment path index");
      incrementPathIndex();
    }
    path = getCurrentPath();
  } else {
    path = status_.backward_path;
  }

  output.path = std::make_shared<PathWithLaneId>(path);
  output.turn_signal_info =
    calcTurnSignalInfo(status_.pull_out_path.start_pose, status_.pull_out_path.end_pose);

  publishDebugData();

  return output;
}

CandidateOutput PullOutModule::planCandidate() const { return CandidateOutput{}; }

std::shared_ptr<PullOutPlannerBase> PullOutModule::getCurrentPlanner() const
{
  for (const auto & planner : pull_out_planners_) {
    if (status_.planner_type == planner->getPlannerType()) {
      return planner;
    }
  }
  return nullptr;
}

PathWithLaneId PullOutModule::getFullPath() const
{
  const auto pull_out_planner = getCurrentPlanner();
  if (pull_out_planner == nullptr) {
    return PathWithLaneId{};
  }

  // combine partial pull out path
  PathWithLaneId pull_out_path;
  for (const auto & partial_path : status_.pull_out_path.partial_paths) {
    pull_out_path.points.insert(
      pull_out_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  if (status_.back_finished) {
    // not need backward path or finish it
    return pull_out_path;
  }

  // concat back_path and pull_out_path and
  auto full_path = status_.backward_path;
  full_path.points.insert(
    full_path.points.end(), pull_out_path.points.begin(), pull_out_path.points.end());
  return full_path;
}

BehaviorModuleOutput PullOutModule::planWaitingApproval()
{
  BehaviorModuleOutput output;
  const auto current_lanes = getCurrentLanes();
  const auto shoulder_lanes = pull_out_utils::getPullOutLanes(current_lanes, planner_data_);

  const auto candidate_path = status_.back_finished ? getCurrentPath() : status_.backward_path;
  auto stop_path = candidate_path;
  for (auto & p : stop_path.points) {
    p.point.longitudinal_velocity_mps = 0.0;
  }

  output.path = std::make_shared<PathWithLaneId>(stop_path);
  output.turn_signal_info =
    calcTurnSignalInfo(status_.pull_out_path.start_pose, status_.pull_out_path.end_pose);
  output.path_candidate = std::make_shared<PathWithLaneId>(candidate_path);

  waitApproval();
  // requset approval when stopped at the corresponding point, so distance is 0
  updateRTCStatus(0.0);

  publishDebugData();
  return output;
}

void PullOutModule::resetStatus()
{
  PullOutStatus initial_status;
  status_ = initial_status;
}

ParallelParkingParameters PullOutModule::getGeometricPullOutParameters() const
{
  ParallelParkingParameters params{};

  params.th_arrived_distance = parameters_.th_arrived_distance;
  params.th_stopped_velocity = parameters_.th_stopped_velocity;
  params.arc_path_interval = parameters_.arc_path_interval;
  params.departing_velocity = parameters_.geometric_pull_out_velocity;
  params.departing_lane_departure_margin = parameters_.lane_departure_margin;
  params.max_steer_angle = parameters_.pull_out_max_steer_angle;

  return params;
}

void PullOutModule::incrementPathIndex()
{
  status_.current_path_idx =
    std::min(status_.current_path_idx + 1, status_.pull_out_path.partial_paths.size() - 1);
}

PathWithLaneId PullOutModule::getCurrentPath() const
{
  return status_.pull_out_path.partial_paths.at(status_.current_path_idx);
}

void PullOutModule::updatePullOutStatus()
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & common_parameters = planner_data_->parameters;
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & goal_pose = planner_data_->route_handler->getGoalPose();

  const auto current_lanes = getCurrentLanes();
  status_.current_lanes = current_lanes;

  // Get pull_out lanes
  const auto pull_out_lanes = pull_out_utils::getPullOutLanes(current_lanes, planner_data_);
  status_.pull_out_lanes = pull_out_lanes;

  // search pull out start candidates backward
  std::vector<Pose> pull_out_start_candidates;
  {
    if (parameters_.enable_back) {
      // the first element is current_pose
      pull_out_start_candidates = searchBackedPoses();
    } else {
      // pull_out_start candidate is only current pose
      pull_out_start_candidates.push_back(current_pose);
    }
  }

  bool found_safe_path = false;
  for (const auto & pull_out_start_pose : pull_out_start_candidates) {
    // plan with each planner
    for (const auto & planner : pull_out_planners_) {
      planner->setPlannerData(planner_data_);
      const auto pull_out_path = planner->plan(pull_out_start_pose, goal_pose);
      if (pull_out_path) {  // found safe path
        found_safe_path = true;
        status_.pull_out_path = *pull_out_path;
        status_.pull_out_start_pose = pull_out_start_pose;
        status_.planner_type = planner->getPlannerType();
        break;
      }
    }
    if (found_safe_path) {
      break;
    }
    // pull out start pose is not current_pose(index > 0), so need back.
    status_.back_finished = false;
  }

  if (!found_safe_path) {
    RCLCPP_ERROR_THROTTLE(getLogger(), *clock_, 5000, "Not found safe pull out path");
    status_.is_safe = false;
    return;
  }

  checkBackFinished();
  if (!status_.back_finished) {
    status_.backward_path = pull_out_utils::getBackwardPath(
      *route_handler, pull_out_lanes, current_pose, status_.pull_out_start_pose,
      parameters_.backward_velocity);
    status_.backward_path.drivable_area = util::generateDrivableArea(
      status_.backward_path, pull_out_lanes, common_parameters.drivable_area_resolution,
      common_parameters.vehicle_length, planner_data_);
  }

  // Update status
  status_.is_safe = found_safe_path;
  status_.lane_follow_lane_ids = util::getIds(current_lanes);
  status_.pull_out_lane_ids = util::getIds(pull_out_lanes);
}

lanelet::ConstLanelets PullOutModule::getCurrentLanes() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto common_parameters = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 5000, "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler->getLaneletSequence(
    current_lane, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length);
}

// make this class?
std::vector<Pose> PullOutModule::searchBackedPoses()
{
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_lanes = getCurrentLanes();
  const auto pull_out_lanes = pull_out_utils::getPullOutLanes(current_lanes, planner_data_);

  // get backward shoulder path
  const auto arc_position_pose = lanelet::utils::getArcCoordinates(pull_out_lanes, current_pose);
  const double check_distance = parameters_.max_back_distance + 30.0;  // buffer
  auto backward_shoulder_path = planner_data_->route_handler->getCenterLinePath(
    pull_out_lanes, arc_position_pose.length - check_distance,
    arc_position_pose.length + check_distance);

  // lateral shift to current_pose
  const double distance_from_center_line = arc_position_pose.distance;
  for (auto & p : backward_shoulder_path.points) {
    p.point.pose = calcOffsetPose(p.point.pose, 0, distance_from_center_line, 0);
  }

  // check collision between footprint and onject at the backed pose
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info_);
  std::vector<Pose> backed_poses;
  for (double back_distance = 0.0; back_distance <= parameters_.max_back_distance;
       back_distance += parameters_.backward_search_resolution) {
    const auto backed_pose = calcLongitudinalOffsetPose(
      backward_shoulder_path.points, current_pose.position, -back_distance);
    if (!backed_pose) {
      continue;
    }

    if (util::checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, *backed_pose, *(planner_data_->dynamic_object),
          parameters_.collision_check_margin)) {
      break;  // poses behind this has a collision, so break.
    }

    backed_poses.push_back(*backed_pose);
  }
  return backed_poses;
}

bool PullOutModule::isInLane(
  const lanelet::ConstLanelet & candidate_lanelet,
  const tier4_autoware_utils::LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (boost::geometry::within(point, candidate_lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

bool PullOutModule::hasFinishedPullOut() const
{
  if (!status_.back_finished) {
    return false;
  }

  // check ego car is close enough to goal pose
  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.current_lanes, current_pose);
  const auto arclength_pull_out_end =
    lanelet::utils::getArcCoordinates(status_.current_lanes, status_.pull_out_path.end_pose);

  // has passed pull out end point
  return arclength_current.length - arclength_pull_out_end.length >
         parameters_.pull_out_finish_judge_buffer;
}

void PullOutModule::checkBackFinished()
{
  // check ego car is close enough to goal pose
  const auto current_pose = planner_data_->self_pose->pose;
  const auto distance =
    tier4_autoware_utils::calcDistance2d(current_pose, status_.pull_out_start_pose);

  const bool is_near = distance < parameters_.th_arrived_distance;
  const double ego_vel = util::l2Norm(planner_data_->self_odometry->twist.twist.linear);
  const bool is_stopped = ego_vel < parameters_.th_stopped_velocity;

  if (!status_.back_finished && is_near && is_stopped) {
    RCLCPP_INFO(getLogger(), "back finished");
    status_.back_finished = true;

    // requst pull_out approval
    waitApproval();
    removeRTCStatus();
    uuid_ = generateUUID();
    // requset approval when stopped at the corresponding point, so distance is 0
    updateRTCStatus(0.0);
    current_state_ = BT::NodeStatus::SUCCESS;  // for breaking loop
  }
}

bool PullOutModule::isStopped()
{
  odometry_buffer_.push_back(planner_data_->self_odometry);
  // Delete old data in buffer
  while (rclcpp::ok()) {
    const auto time_diff = rclcpp::Time(odometry_buffer_.back()->header.stamp) -
                           rclcpp::Time(odometry_buffer_.front()->header.stamp);
    if (time_diff.seconds() < parameters_.th_stopped_time) {
      break;
    }
    odometry_buffer_.pop_front();
  }
  bool is_stopped = true;
  for (const auto & odometry : odometry_buffer_) {
    const double ego_vel = util::l2Norm(odometry->twist.twist.linear);
    if (ego_vel > parameters_.th_stopped_velocity) {
      is_stopped = false;
      break;
    }
  }
  return is_stopped;
}

bool PullOutModule::hasFinishedCurrentPath()
{
  const auto current_path = getCurrentPath();
  const auto current_path_end = current_path.points.back();
  const auto self_pose = planner_data_->self_pose->pose;
  const bool is_near_target = tier4_autoware_utils::calcDistance2d(current_path_end, self_pose) <
                              parameters_.th_arrived_distance;

  return is_near_target && isStopped();
}

TurnSignalInfo PullOutModule::calcTurnSignalInfo(const Pose start_pose, const Pose end_pose) const
{
  TurnSignalInfo turn_signal;

  // turn hazard light when backward driving
  if (!status_.back_finished) {
    turn_signal.hazard_signal.command = HazardLightsCommand::ENABLE;
    turn_signal.signal_distance =
      tier4_autoware_utils::calcDistance2d(start_pose, planner_data_->self_pose->pose);
    return turn_signal;
  }

  // calculate distance to pull_out end on target lanes
  const auto current_lanes = getCurrentLanes();
  const auto arc_position_current_pose =
    lanelet::utils::getArcCoordinates(current_lanes, planner_data_->self_pose->pose);
  const auto arc_position_pull_out_end = lanelet::utils::getArcCoordinates(current_lanes, end_pose);
  const double distance_from_pull_out_end =
    arc_position_current_pose.length - arc_position_pull_out_end.length;

  // turn on right signal until passing pull_out end point
  const double turn_signal_off_buffer = std::min(parameters_.pull_out_finish_judge_buffer, 3.0);
  if (distance_from_pull_out_end < turn_signal_off_buffer) {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  } else {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::DISABLE;
  }

  turn_signal.signal_distance = -distance_from_pull_out_end + turn_signal_off_buffer;

  return turn_signal;
}

void PullOutModule::publishDebugData() const
{
  auto header = planner_data_->route_handler->getRouteHeader();

  PoseStamped pull_out_start_pose;
  pull_out_start_pose.pose = status_.pull_out_start_pose;
  pull_out_start_pose.header = header;
  pull_out_start_pose_pub_->publish(pull_out_start_pose);

  auto full_path_pose_array = util::convertToGeometryPoseArray(getFullPath());
  full_path_pose_array.header = header;
  full_path_pose_array_pub_->publish(full_path_pose_array);
}
}  // namespace behavior_path_planner
