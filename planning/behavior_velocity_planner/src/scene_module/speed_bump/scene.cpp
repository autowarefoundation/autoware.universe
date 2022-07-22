// Copyright 2020 Tier IV, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <scene_module/crosswalk/util.hpp>
#include <scene_module/speed_bump/scene.hpp>
#include <utilization/util.hpp>

#include <cmath>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

using motion_utils::calcSignedArcLength;

SpeedBumpModule::SpeedBumpModule(
  const int64_t module_id, const lanelet::ConstLanelet & speed_bump,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  speed_bump_(speed_bump),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
  dist_to_acc_point_ = std::numeric_limits<double>::max();
  dist_to_slow_point_ = std::numeric_limits<double>::max();
  debug_data_ = speed_bump_util::DebugData();
  is_speed_bump_module_expired_ = false;
  speed_bump_height_ = 0.10;  // [m]
}

bool SpeedBumpModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] tier4_planning_msgs::msg::StopReason * stop_reason)
{
  if (state_ == State::OUT) {
    return false;
  }

  const auto input = *path;

  if (polygon_.outer().empty()) {
    // create polygon
    lanelet::CompoundPolygon3d lanelet_polygon = speed_bump_.polygon3d();
    for (const auto & lanelet_point : lanelet_polygon) {
      polygon_.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
    }
    polygon_.outer().push_back(polygon_.outer().front());
    polygon_ = isClockWise(polygon_) ? polygon_ : inverseClockWise(polygon_);
  }

  // get speed_bump height
  for (const auto & attr : speed_bump_.attributes()) {
    if (attr.first == "height") {
      speed_bump_height_ = attr.second.asDouble().get();
    }
  }

  // debug
  [[maybe_unused]] auto debug_state = "NONE";

  // check current state
  if (dist_to_slow_point_ > 0.0) {
    state_ = State::APPROACH;
    debug_state = "APPROACH";

    debug_data_ = speed_bump_util::DebugData();
    debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

    autoware_auto_planning_msgs::msg::PathWithLaneId slow_path;
    if (!checkSlowArea(input, polygon_, slow_path)) {
      return false;
    }
    *path = slow_path;
  } else if (dist_to_slow_point_ <= 0.0 && dist_to_acc_point_ >= 0.0) {
    state_ = State::INSIDE;
    debug_state = "INSIDE";

    if (path->points.empty()) {
      return false;
    }

    speed_bump_util::setVelocityFromIndex(
      0, behavior_velocity_planner::speed_bump_util::velocityByBumpHeight(speed_bump_height_),
      *path);
  } else if (dist_to_acc_point_ < 0.0) {
    state_ = State::OUT;
    debug_state = "OUT";

    debug_data_ = speed_bump_util::DebugData();
    is_speed_bump_module_expired_ = true;
  }

  // Calculate distance to slow point
  if (!debug_data_.slow_poses.empty()) {
    dist_to_slow_point_ = calcSignedArcLength(
      path->points, planner_data_->current_pose.pose.position,
      debug_data_.slow_poses.at(0).position);
  }

  // Calculate distance to acc point
  if (!debug_data_.acc_poses.empty()) {
    dist_to_acc_point_ = calcSignedArcLength(
      path->points, planner_data_->current_pose.pose.position,
      debug_data_.acc_poses.at(0).position);
  }

  return true;
}

bool SpeedBumpModule::checkSlowArea(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
  const Polygon & speed_bump_polygon, autoware_auto_planning_msgs::msg::PathWithLaneId & output)
{
  output = input;

  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < speed_bump_polygon.outer().size(); ++i) {
    Eigen::Vector3d point;
    point << speed_bump_polygon.outer().at(i).x(), speed_bump_polygon.outer().at(i).y(),
      planner_data_->current_pose.pose.position.z;
    points.push_back(point);
  }
  debug_data_.slow_polygons.push_back(points);
  // ----------------

  // insert slow point
  if (!insertTargetVelocityPoint(
        input, speed_bump_polygon, planner_param_.slow_margin, planner_param_.acceleration_margin,
        speed_bump_height_, *planner_data_, output, debug_data_)) {
    return false;
  }

  return true;
}

}  // namespace behavior_velocity_planner
