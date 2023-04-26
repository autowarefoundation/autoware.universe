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

#include "behavior_path_planner/scene_module/dynamic_avoidance/dynamic_avoidance_module.hpp"

#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_path_planner
{
namespace
{
bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets)
{
  if (target_lanelets.empty()) {
    return false;
  }

  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d object_centroid(object_pos.x, object_pos.y);

  for (const auto & llt : target_lanelets) {
    if (boost::geometry::within(object_centroid, llt.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

std::vector<PredictedObject> getObjectsInLanes(
  const std::vector<PredictedObject> & objects, const lanelet::ConstLanelets & target_lanes)
{
  std::vector<PredictedObject> target_objects;
  for (const auto & object : objects) {
    if (isCentroidWithinLanelets(object, target_lanes)) {
      target_objects.push_back(object);
    }
  }

  return target_objects;
}

geometry_msgs::msg::Point toGeometryPoint(const tier4_autoware_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_obj_point;
  geom_obj_point.x = point.x();
  geom_obj_point.y = point.y();
  return geom_obj_point;
}
}  // namespace

#ifdef USE_OLD_ARCHITECTURE
DynamicAvoidanceModule::DynamicAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters)
: SceneModuleInterface{name, node, createRTCInterfaceMap(node, name, {""})},
  parameters_{std::move(parameters)}
#else
DynamicAvoidanceModule::DynamicAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map}, parameters_{std::move(parameters)}
#endif
{
}

bool DynamicAvoidanceModule::isExecutionRequested() const
{
  RCLCPP_INFO(getLogger(), "DYNAMIC AVOIDANCE isExecutionRequested.");

  // check if the ego is driving forward
  const auto is_driving_forward = [&]() {
    if (!getPreviousModuleOutput().path || getPreviousModuleOutput().path->points.size() < 2) {
      return false;
    }
    const auto is_driving_forward =
      motion_utils::isDrivingForward(getPreviousModuleOutput().path->points);
    if (!is_driving_forward) {
      return false;
    }
    return *is_driving_forward;
  }();

  // check if the planner is already running
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  // check if there is target objects to avoid
  return !target_objects_.empty();
}

bool DynamicAvoidanceModule::isExecutionReady() const
{
  RCLCPP_INFO(getLogger(), "DYNAMIC AVOIDANCE isExecutionReady.");
  return true;
}

void DynamicAvoidanceModule::updateData()
{
  std::cerr << "updateData" << std::endl;
  target_objects_ = calcTargetObjects();
}

ModuleStatus DynamicAvoidanceModule::updateState()
{
  const bool has_avoidance_target = !target_objects_.empty();

  if (!has_avoidance_target) {
    current_state_ = ModuleStatus::SUCCESS;
  } else {
    current_state_ = ModuleStatus::RUNNING;
  }

  return current_state_;
}

BehaviorModuleOutput DynamicAvoidanceModule::plan()
{
  // generate reference path
  auto reference_path = utils::generateCenterLinePath(planner_data_);

  // TODO(murooka) implement here
  // generate drivable area
  const auto & p = planner_data_->parameters;

  const auto current_lanes = utils::getCurrentLanes(planner_data_);
  const auto drivable_lanes = utils::generateDrivableLanes(current_lanes);
  const auto target_drivable_lanes =
    getNonOverlappingExpandedLanes(*reference_path, drivable_lanes);

  // for old arhictecture
  utils::generateDrivableArea(
    *reference_path, target_drivable_lanes, p.vehicle_length, planner_data_);

  std::vector<tier4_autoware_utils::Polygon2d> obstacle_polys;
  for (const auto & object : target_objects_) {
    const auto obstacle_poly = calcDynamicObstaclesPolygon(*reference_path, object);
    obstacle_polys.push_back(obstacle_poly);
  }

  BehaviorModuleOutput output;
  output.path = reference_path;
  output.reference_path = reference_path;

  // for new arhictecture
  output.drivable_area_info.drivable_lanes = target_drivable_lanes;
  output.drivable_area_info.obstacle_polys = obstacle_polys;

  return output;
}

CandidateOutput DynamicAvoidanceModule::planCandidate() const
{
  auto candidate_path = utils::generateCenterLinePath(planner_data_);
  return CandidateOutput(*candidate_path);
}

BehaviorModuleOutput DynamicAvoidanceModule::planWaitingApproval()
{
  BehaviorModuleOutput out = plan();
  return out;
}

std::vector<DynamicAvoidanceModule::DynamicAvoidanceObject>
DynamicAvoidanceModule::calcTargetObjects() const
{
  // calculate target lanes to filter obstacles
  const auto target_lanes = getAdjacentLanes(100.0, 5.0);

  // calculate obstacles for dynamic avoidance
  const auto & predicted_objects = planner_data_->dynamic_object->objects;
  const auto target_predicted_objects = getObjectsInLanes(predicted_objects, target_lanes);
  std::cerr << target_predicted_objects.size() << std::endl;

  // convert predicted objects to dynamic avoidance objects
  std::vector<DynamicAvoidanceObject> target_avoidance_objects;
  for (const auto & predicted_object : target_predicted_objects) {
    target_avoidance_objects.push_back(DynamicAvoidanceObject(predicted_object));
  }

  return target_avoidance_objects;
}

lanelet::ConstLanelets DynamicAvoidanceModule::getAdjacentLanes(
  const double forward_distance, const double backward_distance) const
{
  const auto & rh = planner_data_->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(getEgoPose(), &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};
  }

  const auto ego_succeeding_lanes =
    rh->getLaneletSequence(current_lane, getEgoPose(), backward_distance, forward_distance);

  lanelet::ConstLanelets target_lanes;
  for (const auto & lane : ego_succeeding_lanes) {
    const auto opt_left_lane = rh->getLeftLanelet(lane);
    if (opt_left_lane) {
      target_lanes.push_back(opt_left_lane.get());
    }

    const auto opt_right_lane = rh->getRightLanelet(lane);
    if (opt_right_lane) {
      target_lanes.push_back(opt_right_lane.get());
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (!right_opposite_lanes.empty()) {
      target_lanes.push_back(right_opposite_lanes.front());
    }
  }

  return target_lanes;
}

/*
void boundDynamicObstacles(PathWithLaneId & path, const DynamicAvoidanceObject & object)
{
  // calculate lat offset
  const double lat_offset = motion_utils::calcLateralOffset(reference_path, object.pose.position);
  const bool is_left = 0.0 < lat_offset;

  auto & target_bound = is_left ? reference_path.left_bound : reference_path.right_bound;
  const size_t obj_seg_idx = motion_utils::findNearsetSegmentIndex(target_bound,
object.pose.position);

  // calculate front and back point
  const auto obj_points = tier4_autoware_utils::toPolygon2d(object.shape);
  std::vector<double> obj_lon_offset_vec;
  for (const auto & obj_point : obj_points.outer()) {
    const lon_offset = motion_utils::calcLongitudinalOffsetToSement(target_bound, obj_seg_idx,
obj_point); obj_lon_offset_vec.push_back(lon_offset);
  }
  const auto itr = std::minmax_element(obj_lon_offset_vec.begin(), obj_lon_offset_vec.end());
  const size_t front_idx = std::distance(obj_lon_offset_vec.begin(), itr.first);
  const size_t back_idx = std::distance(obj_lon_offset_vec.begin(), itr.second);
  const auto obj_front_pos = obj_points.outer().at(front_idx);
  const auto obj_back_pos = obj_points.outer().at(back_idx);

  const size_t front_seg_idx = motion_utils::insertTargetPoint(-3.0, obj_front_pos, target_bound);
  const size_t back_seg_idx = motion_utils::insertTargetPoint(3.0, obj_back_pos, target_bound);

  for (size_t i = front_seg_idx; i < back_seg_idx) {
    target_bound.at(i) =
  }
}
*/

tier4_autoware_utils::Polygon2d DynamicAvoidanceModule::calcDynamicObstaclesPolygon(
  const PathWithLaneId & path, const DynamicAvoidanceObject & object)
{
  // calculate lat offset
  const size_t obj_seg_idx =
    motion_utils::findNearestSegmentIndex(path.points, object.pose.position);
  const double obj_lat_offset =
    motion_utils::calcLateralOffset(path.points, object.pose.position, obj_seg_idx);
  const bool is_left = 0.0 < obj_lat_offset;

  const auto [min_obj_lat_offset, min_obj_lat_offset_point] = [&]() {
    const auto obj_points = tier4_autoware_utils::toPolygon2d(object.pose, object.shape);
    std::vector<double> obj_lat_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const size_t obj_point_seg_idx =
        motion_utils::findNearestSegmentIndex(path.points, geom_obj_point);
      const double obj_point_lat_offset =
        motion_utils::calcLateralOffset(path.points, geom_obj_point, obj_point_seg_idx);
      obj_lat_offset_vec.push_back(obj_point_lat_offset);
    }
    const size_t min_obj_lat_offset_idx = std::distance(
      obj_lat_offset_vec.begin(),
      std::min_element(obj_lat_offset_vec.begin(), obj_lat_offset_vec.end()));

    return std::make_pair(
      obj_lat_offset_vec.at(min_obj_lat_offset_idx),
      toGeometryPoint(obj_points.outer().at(min_obj_lat_offset_idx)));
  }();

  // calculate object bound
  const size_t obj_front_seg_idx =
    motion_utils::findNearestSegmentIndex(path.points, min_obj_lat_offset_point);
  std::vector<geometry_msgs::msg::Point> obj_bound_points;
  for (size_t i = obj_front_seg_idx; i < obj_front_seg_idx; ++i) {
    obj_bound_points.push_back(tier4_autoware_utils::calcOffsetPose(
                                 path.points.at(i).point.pose, 0.0, min_obj_lat_offset, 0.0)
                                 .position);
  }

  // convert obj_bound_points to obj_polygon
  tier4_autoware_utils::Polygon2d obj_poly;
  for (size_t i = 0; i < obj_bound_points.size(); ++i) {
    const auto obj_poly_point =
      tier4_autoware_utils::Point2d(obj_bound_points.at(i).x, obj_bound_points.at(i).y);
    obj_poly.outer().push_back(obj_poly_point);
  }

  /*
  auto & target_bound = is_left ? path.left_bound : path.right_bound;
  const size_t obj_seg_idx = motion_utils::findNearsetSegmentIndex(target_bound,
  object.pose.position);

  // calculate front and back point
  std::vector<double> obj_lon_offset_vec;
  for (const auto & obj_point : obj_points.outer()) {
    const lon_offset = motion_utils::calcLongitudinalOffsetToSement(target_bound, obj_seg_idx,
  obj_point); obj_lon_offset_vec.push_back(lon_offset);
  }
  */
}
}  // namespace behavior_path_planner
