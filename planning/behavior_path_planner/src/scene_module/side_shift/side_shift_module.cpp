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

#include <memory>
#include <string>

#include "opencv2/opencv.hpp"

#include "lanelet2_extension/utility/utilities.hpp"
#include "tf2/utils.h"

#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"
#include "behavior_path_planner/scene_module/side_shift/util.hpp"
#include "behavior_path_planner/utilities.hpp"

namespace behavior_path_planner
{
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;

SideShiftModule::SideShiftModule(
  const std::string & name, rclcpp::Node & node,
  const SideShiftParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  using std::placeholders::_1;

  lateral_offset_subscriber_ =
    node.create_subscription<LateralOffset>(
    "~/input/side_shift_length", 1, std::bind(&SideShiftModule::onLateralOffset, this, _1));
}

void SideShiftModule::initVariables()
{
  refined_path_ = PathWithLaneId{};
  reference_path_ = std::make_shared<PathWithLaneId>();
  start_pose_reset_request_ = false;
  drivable_area_shrink_request_ = false;
  start_avoid_pose_ = Pose{};
  lateral_offset_ = 0.0;
  prev_planned_lateral_offset_ = 0.0;
  drivable_area_extention_width_ = 0.0;
  is_approval_needed_ = true;  // needs approval at first
}

void SideShiftModule::onEntry()
{
  // write me... (Don't initialize variables, otherwise lateral offset gets zero on entry.)
  start_avoid_pose_ = Pose{};
  refined_path_ = PathWithLaneId{};
}

void SideShiftModule::onExit()
{
  // write me...
  initVariables();

  current_state_ = BT::NodeStatus::IDLE;
}

void SideShiftModule::setParameters(const SideShiftParameters & parameters)
{
  parameters_ = parameters;
}

bool SideShiftModule::isExecutionRequested() const
{
  // If the desired offset has a non-zero value, return true as we want to execute the plan.

  const bool has_request = !isAlmostZero(lateral_offset_);
  RCLCPP_DEBUG_STREAM(
    getLogger(),
    "ESS::isExecutionRequested() : " << std::boolalpha << has_request);

  return has_request;
}

bool SideShiftModule::isExecutionReady() const
{
  return true;  // TODO(Horibe) is it ok to say "always safe"?
}

BT::NodeStatus SideShiftModule::updateState()
{
  // Never return the FAILURE. When the desired offset is zero and the vehicle is in the original
  // drivable area,this module can stop the computation and return SUCCESS.

  const bool no_request = isAlmostZero(lateral_offset_);
  const bool on_reference = isVehicleInDrivableArea(reference_path_->drivable_area);

  RCLCPP_DEBUG(
    getLogger(), "ESS::updateState() : no_request = %d, on_reference = %d", no_request,
    on_reference);

  if (no_request && on_reference) {
    current_state_ = BT::NodeStatus::SUCCESS;
  } else {
    current_state_ = BT::NodeStatus::RUNNING;
  }

  return current_state_;
}

void SideShiftModule::updateData()
{
  reference_path_ = util::generateCenterLinePath(planner_data_);

  const auto & route_handler = planner_data_->route_handler;
  const auto & p = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(planner_data_->self_pose->pose, &current_lane)) {
    RCLCPP_ERROR(getLogger(), "failed to find closest lanelet within route!!!");
  }

  // For current_lanes with desired length
  current_lanelets_ = route_handler->getLaneletSequence(
    current_lane, planner_data_->self_pose->pose, p.backward_path_length, p.forward_path_length);
}

BehaviorModuleOutput SideShiftModule::plan()
{
  // TODO(Horibe) Apply resampling since it is too sparse.

  refined_path_ = refinePath(
    *reference_path_, lateral_offset_, start_pose_reset_request_, drivable_area_shrink_request_,
    drivable_area_extention_width_, start_avoid_pose_);

  prev_planned_lateral_offset_ = lateral_offset_;

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(refined_path_);

  return output;
}

PathWithLaneId SideShiftModule::planCandidate() const
{
  auto start_pose_reset_request = start_pose_reset_request_;
  auto drivable_area_shrink_request = drivable_area_shrink_request_;
  auto drivable_area_extention_width = drivable_area_extention_width_;
  auto start_avoid_pose = start_avoid_pose_;

  const auto path = refinePath(
    *reference_path_, lateral_offset_, start_pose_reset_request, drivable_area_shrink_request,
    drivable_area_extention_width, start_avoid_pose);

  return path;
}

BehaviorModuleOutput SideShiftModule::planWaitingApproval()
{
  BehaviorModuleOutput out;
  const auto lateral_offset = prev_planned_lateral_offset_;  // use previous plan!
  auto start_pose_reset_request = false;
  auto drivable_area_shrink_request = false;
  auto drivable_area_extention_width = drivable_area_extention_width_;
  auto start_avoid_pose = start_avoid_pose_;

  refined_path_ = refinePath(
    *reference_path_, lateral_offset, start_pose_reset_request, drivable_area_shrink_request,
    drivable_area_extention_width, start_avoid_pose);

  out.path = std::make_shared<PathWithLaneId>(refined_path_);
  out.path_candidate = std::make_shared<PathWithLaneId>(planCandidate());

  return out;
}

void SideShiftModule::onLateralOffset(const LateralOffset::ConstSharedPtr lateral_offset_msg)
{
  const double new_lateral_offset = lateral_offset_msg->lateral_offset;

  // offset is not changed.
  if (std::abs(lateral_offset_ - new_lateral_offset) < 1e-4) {
    lateral_offset_ = new_lateral_offset;  // update lateral offset
    return;
  }

  // offset is changed.
  approval_handler_.waitApproval();  // requires external approval when offset is changed

  start_pose_reset_request_ = true;
  if (!drivable_area_shrink_request_) {
    if ((std::abs(lateral_offset_) > std::abs(new_lateral_offset))) {
      drivable_area_shrink_request_ = true;
      drivable_area_extention_width_ = lateral_offset_;
    } else {
      drivable_area_extention_width_ = new_lateral_offset;
    }
  }

  lateral_offset_ = new_lateral_offset;
}

PathWithLaneId SideShiftModule::refinePath(
  const PathWithLaneId & input_path, double lateral_offset, bool & start_pose_reset_request,
  bool & drivable_area_shrink_request, double & drivable_area_extention_width,
  Pose & start_avoid_pose) const
{
  using autoware_utils::findNearestIndex;

  if (refined_path_.points.empty()) {return input_path;}

  auto output_path = input_path;
  const auto & param = planner_data_->parameters;

  // Find the nearest path index to current pose
  const size_t nearest_idx =
    findNearestIndex(output_path.points, planner_data_->self_pose->pose.position);

  // Get pose in path starting avoidance
  if (start_pose_reset_request) {
    const double current_velocity = planner_data_->self_velocity->twist.linear.x;
    const double start_distance = std::max(
      parameters_.min_fix_distance,
      current_velocity * parameters_.start_avoid_sec + param.base_link2front);
    if (getStartAvoidPose(output_path, start_distance, nearest_idx, &start_avoid_pose)) {
      start_pose_reset_request = false;
    }
  }

  // Find the nearest path index to starting avoidance pose
  const size_t start_avoid_idx = findNearestIndex(output_path.points, start_avoid_pose.position);

  // Refine path
  for (size_t idx = 0; idx < output_path.points.size(); ++idx) {
    auto & pt = output_path.points.at(idx).point;
    if (idx < start_avoid_idx) {
      // Set position from previous published refined path for continuity
      const size_t refined_nearest_idx = findNearestIndex(refined_path_.points, pt.pose.position);
      pt.pose.position = refined_path_.points.at(refined_nearest_idx).point.pose.position;
    } else if (!start_pose_reset_request) {
      // Add lateral offset
      const auto input_pt = input_path.points.at(idx).point;
      double yaw = tf2::getYaw(input_pt.pose.orientation);
      pt.pose.position.x -= lateral_offset * std::sin(yaw);
      pt.pose.position.y += lateral_offset * std::cos(yaw);
    }
  }

  // Reset orientation
  setOrientation(&output_path);

  // Inside outside check if offset is decreased
  if (drivable_area_shrink_request) {
    auto path_with_extended_drivable_area = output_path;
    extendDrivableArea(
      path_with_extended_drivable_area, lateral_offset, *(planner_data_->route_handler));
    if (isVehicleInDrivableArea(path_with_extended_drivable_area.drivable_area)) {
      output_path.drivable_area = path_with_extended_drivable_area.drivable_area;
      drivable_area_extention_width = lateral_offset;
      drivable_area_shrink_request = false;
      return output_path;
    }
  }

  // Refine drivable area
  extendDrivableArea(output_path, drivable_area_extention_width, *(planner_data_->route_handler));

  return output_path;
}

void SideShiftModule::extendDrivableArea(
  PathWithLaneId & path, double lateral_offset,
  [[maybe_unused]] const RouteHandler & route_handler) const
{
  const double resolution = path.drivable_area.info.resolution;
  const double width = path.drivable_area.info.width * resolution;
  const double height = path.drivable_area.info.height * resolution;
  const double vehicle_length = planner_data_->parameters.vehicle_length;

  const double left_extend = lateral_offset > 0.0 ? lateral_offset : 0.0;
  const double right_extend = lateral_offset < 0.0 ? lateral_offset : 0.0;
  const auto extended_lanelets =
    lanelet::utils::getExpandedLanelets(current_lanelets_, left_extend, right_extend);

  path.drivable_area = util::generateDrivableArea(
    extended_lanelets, *(planner_data_->self_pose), width, height, resolution, vehicle_length,
    *(planner_data_->route_handler));
}

bool SideShiftModule::isVehicleInDrivableArea(const OccupancyGrid & drivable_area) const
{
  // Get grid origin
  PoseStamped grid_origin{};
  grid_origin.pose = drivable_area.info.origin;
  grid_origin.header = drivable_area.header;

  // Get transform
  tf2::Stamped<tf2::Transform> tf_grid2map, tf_map2grid;
  tf2::convert(grid_origin, tf_grid2map);
  tf_map2grid.setData(tf_grid2map.inverse());
  const auto geom_tf_map2grid = tf2::toMsg(tf_map2grid);

  constexpr uint8_t FREE_SPACE = 0;
  constexpr uint8_t OCCUPIED_SPACE = 100;

  // Convert occupancy grid to image
  cv::Mat cv_image(
    drivable_area.info.width, drivable_area.info.height, CV_8UC1, cv::Scalar(OCCUPIED_SPACE));
  behavior_path_planner::util::occupancyGridToImage(drivable_area, &cv_image);

  // Convert vehicle vertices to cv point
  cv::Point cv_front_left, cv_front_right, cv_rear_left, cv_rear_right;
  {
    // Get current base_link informations
    const auto current_pose = *(planner_data_->self_pose);
    const auto base_link = current_pose.pose.position;
    const double yaw = tf2::getYaw(current_pose.pose.orientation);

    // Get vehicle informations
    const auto planner_param = planner_data_->parameters;
    const double margin = parameters_.inside_outside_judge_margin;
    const double l_front = planner_param.base_link2front + margin;
    const double l_rear = -(planner_param.base_link2rear + margin);
    const double l_side = 0.5 * planner_param.vehicle_width + margin;

    // Calculate vertices on grid coordinate
    auto transformToGridLocal = [&](const double l1, const double l2) {
        return transformToGrid(base_link, l1, l2, yaw, geom_tf_map2grid);
      };
    auto grid_front_left = transformToGridLocal(l_front, l_side);
    auto grid_front_right = transformToGridLocal(l_front, -l_side);
    auto grid_rear_left = transformToGridLocal(l_rear, l_side);
    auto grid_rear_right = transformToGridLocal(l_rear, -l_side);

    // Convert to cv coordinate
    auto toCVPointLocal = [&](const Point & p) {
        return behavior_path_planner::util::toCVPoint(
          p, parameters_.drivable_area_width, parameters_.drivable_area_height,
          parameters_.drivable_area_resolution);
      };
    cv_front_left = toCVPointLocal(grid_front_left);
    cv_front_right = toCVPointLocal(grid_front_right);
    cv_rear_left = toCVPointLocal(grid_rear_left);
    cv_rear_right = toCVPointLocal(grid_rear_right);
  }

  auto isFreeSpace = [&](const cv::Point & p) {return cv_image.at<uint8_t>(p) == FREE_SPACE;};
  bool is_vehicle_on_freespace = isFreeSpace(cv_front_left) && isFreeSpace(cv_front_right) &&
    isFreeSpace(cv_rear_left) && isFreeSpace(cv_rear_right);
  return is_vehicle_on_freespace;
}

}  // namespace behavior_path_planner
