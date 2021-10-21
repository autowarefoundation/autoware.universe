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

#include <functional>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "autoware_utils/geometry/geometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "utilization/path_utilization.hpp"
#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
ROAD_TYPE getCurrentRoadType(
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  const auto logger{rclcpp::get_logger("behavior_velocity_planner").get_child("occlusion_spot")};
  rclcpp::Clock clock{RCL_ROS_TIME};
  occlusion_spot_utils::ROAD_TYPE road_type;
  std::string location;
  if (
    current_lanelet.hasAttribute(lanelet::AttributeNamesString::Subtype) &&
    current_lanelet.attribute(lanelet::AttributeNamesString::Subtype) ==
    lanelet::AttributeValueString::Highway)
  {
    location = "highway";
  } else {
    location = current_lanelet.attributeOr("location", "else");
  }
  RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "location: " << location);
  if (location == "urban" || location == "public") {
    road_type = occlusion_spot_utils::ROAD_TYPE::PUBLIC;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "public road: " << location);
  } else if (location == "private") {
    road_type = occlusion_spot_utils::ROAD_TYPE::PRIVATE;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "private road");
  } else if (location == "highway") {
    road_type = occlusion_spot_utils::ROAD_TYPE::HIGHWAY;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "highway road");
  } else {
    road_type = occlusion_spot_utils::ROAD_TYPE::UNKNOWN;
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, 3000, "unknown road");
  }
  return road_type;
}

void calcVelocityAndHeightToPossibleCollision(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  std::vector<PossibleCollisionInfo> & possible_collisions)
{
  if (possible_collisions.empty()) {return;}
  std::sort(
    possible_collisions.begin(), possible_collisions.end(),
    [](PossibleCollisionInfo pc1, PossibleCollisionInfo pc2) {
      return pc1.arc_lane_dist_at_collision.length < pc2.arc_lane_dist_at_collision.length;
    });

  // get interpolated value between (s0,v0) - (s1,value) - (s2,v2)
  auto getInterpolatedValue = [](double s0, double v0, double s1, double s2, double v2) {
      if (s2 - s0 < std::numeric_limits<float>::min()) {return v0;}
      return v0 + (v2 - v0) * (s1 - s0) / (s2 - s0);
    };
  // insert path point orientation to possible collision
  size_t collision_index = 0;
  double dist_along_path_point = 0;
  double dist_along_next_path_point = 0;
  for (size_t idx = 0; idx < path.points.size() - 1; idx++) {
    auto p_prev = path.points[idx].point;
    auto p_next = path.points[idx + 1].point;
    const double dist_to_col =
      possible_collisions.at(collision_index).arc_lane_dist_at_collision.length;
    dist_along_next_path_point +=
      autoware_utils::calcDistance2d(p_prev.pose.position, p_next.pose.position);
    // process if nearest possible collision is between current and next path point
    if (dist_along_path_point < dist_to_col) {
      for (; collision_index < possible_collisions.size(); collision_index++) {
        const double current_dist2col =
          possible_collisions[collision_index].arc_lane_dist_at_collision.length;
        possible_collisions[collision_index].collision_path_point.twist.linear.x =
          getInterpolatedValue(
          dist_along_path_point, p_prev.twist.linear.x, dist_to_col, dist_along_next_path_point,
          p_next.twist.linear.x);
        const double height = getInterpolatedValue(
          dist_along_path_point, p_prev.pose.position.z, dist_to_col, dist_along_next_path_point,
          p_next.pose.position.z);
        // height is used to visualize marker correctly
        possible_collisions[collision_index].collision_path_point.pose.position.z = height;
        possible_collisions[collision_index].intersection_pose.position.z = height;
        possible_collisions[collision_index].obstacle_info.position.z = height;
        // break searching if dist to collision is farther than next path point
        if (dist_along_next_path_point < current_dist2col) {break;}
      }
      if (collision_index == possible_collisions.size()) {break;}
    }
    dist_along_path_point = dist_along_next_path_point;
  }
}

autoware_planning_msgs::msg::Path toPath(
  const autoware_planning_msgs::msg::PathWithLaneId & path_with_id, const int first_index)
{
  autoware_planning_msgs::msg::Path path;
  for (size_t i = first_index; i < path_with_id.points.size(); i++) {
    path.points.push_back(path_with_id.points[i].point);
  }
  return path;
}

lanelet::ConstLanelet buildPathLanelet(
  const autoware_planning_msgs::msg::PathWithLaneId & path, int first_index, double max_range)
{
  autoware_planning_msgs::msg::Path converted_path =
    filterLitterPathPoint(toPath(path, first_index));
  if (converted_path.points.empty()) {return lanelet::ConstLanelet();}
  autoware_planning_msgs::msg::Path interpolated_path = interpolatePath(
    converted_path, max_range,
    2.0);
  lanelet::BasicLineString2d path_line;
  path_line.reserve(interpolated_path.points.size());
  // skip last point that will creates extra ordinary path
  for (size_t i = 0; i < interpolated_path.points.size() - 1; ++i) {
    const auto & pose_i = interpolated_path.points[i].pose;
    path_line.emplace_back(pose_i.position.x, pose_i.position.y);
    if (
      autoware_utils::calcDistance2d(
        path.points[first_index].point.pose.position, pose_i.position) > max_range)
    {
      break;
    }
  }
  // set simplified line to lanelet
  lanelet::BasicLineString2d simplified_line;
  boost::geometry::simplify(path_line, simplified_line, 0.1);
  lanelet::Points3d path_points;
  path_points.reserve(simplified_line.size());
  for (const auto & p : simplified_line) {
    path_points.emplace_back(lanelet::InvalId, p.x(), p.y(), 0.0);
  }
  lanelet::LineString3d centerline(lanelet::InvalId, path_points);
  lanelet::Lanelet path_lanelet(lanelet::InvalId);
  path_lanelet.setCenterline(centerline);
  return lanelet::ConstLanelet(path_lanelet);
}

void createPossibleCollisionBehindParkedVehicle(
  std::vector<PossibleCollisionInfo> & possible_collisions,
  const autoware_planning_msgs::msg::PathWithLaneId & path, const PlannerParam & param,
  const int first_path_index,
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr & dyn_obj_arr)
{
  lanelet::ConstLanelet path_lanelet =
    buildPathLanelet(path, first_path_index, param.detection_area_length);
  if (path_lanelet.centerline2d().empty()) {return;}
  std::vector<lanelet::ArcCoordinates> arcs;
  auto calcSignedArcDistance = [](const double lateral_distance, const double offset) {
      if (lateral_distance < 0) {return lateral_distance + offset;}
      return lateral_distance - offset;
    };
  const double half_vehicle_width = 0.5 * param.vehicle_info.vehicle_width;
  for (const auto & dyn : dyn_obj_arr->objects) {
    // consider if dynamic object is a car or bus or truck
    if (
      dyn.semantic.type != autoware_perception_msgs::msg::Semantic::CAR &&
      dyn.semantic.type != autoware_perception_msgs::msg::Semantic::TRUCK &&
      dyn.semantic.type != autoware_perception_msgs::msg::Semantic::BUS)
    {
      continue;
    }
    const auto & state = dyn.state;
    // ignore if vehicle is moving
    if (state.twist_covariance.twist.linear.x > param.stuck_vehicle_vel) {continue;}
    const geometry_msgs::msg::Point & p = dyn.state.pose_covariance.pose.position;
    const geometry_msgs::msg::Quaternion & q = dyn.state.pose_covariance.pose.orientation;
    lanelet::BasicPoint2d obj_point;
    obj_point[0] = p.x;
    obj_point[1] = p.y;
    lanelet::ArcCoordinates arc_lane_point_at_occlusion =
      lanelet::geometry::toArcCoordinates(path_lanelet.centerline2d(), obj_point);
    // add a half size of car to arc length as behind car
    arc_lane_point_at_occlusion.length += dyn.shape.dimensions.x * 0.5;
    // signed lateral distance used to convert obstacle position
    double signed_lateral_distance =
      calcSignedArcDistance(arc_lane_point_at_occlusion.distance, half_vehicle_width);
    arc_lane_point_at_occlusion.distance =
      std::abs(arc_lane_point_at_occlusion.distance) - 0.5 * dyn.shape.dimensions.y;
    // ignore if obstacle is not within ego base link to front or path and not farther than 2.0[m]
    if (
      arc_lane_point_at_occlusion.length < param.vehicle_info.baselink_to_front ||
      arc_lane_point_at_occlusion.distance < half_vehicle_width ||
      param.lateral_distance_thr + half_vehicle_width < arc_lane_point_at_occlusion.distance)
    {
      continue;
    }
    // subtract vehicle width from lateral distance to occlusion point
    double lateral_dist_at_collision_point =
      arc_lane_point_at_occlusion.distance - half_vehicle_width;
    // subtract  baselink_to_front from longitudinal distance to occlusion point
    double longitudinal_dist_at_collision_point =
      arc_lane_point_at_occlusion.length - param.vehicle_info.baselink_to_front;
    // collision point at baselink
    lanelet::BasicPoint2d collision_point = lanelet::geometry::fromArcCoordinates(
      path_lanelet.centerline2d(), {longitudinal_dist_at_collision_point, 0.0});
    geometry_msgs::msg::Point search_point;
    search_point.x = collision_point[0];
    search_point.y = collision_point[1];
    search_point.z = 0;
    double path_angle = lanelet::utils::getLaneletAngle(path_lanelet, search_point);
    // ignore if angle is more different than 10[degree]
    double obj_angle = tf2::getYaw(q);
    if (std::abs(path_angle - obj_angle) > param.angle_thr) {continue;}
    lanelet::ArcCoordinates arc_lane_dist_at_collision;
    arc_lane_dist_at_collision.length = longitudinal_dist_at_collision_point;
    arc_lane_dist_at_collision.distance = lateral_dist_at_collision_point;

    autoware_planning_msgs::msg::PathPoint collision_path_point;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, path_angle);
    collision_path_point.pose.position.x = collision_point[0];
    collision_path_point.pose.position.y = collision_point[1];
    collision_path_point.pose.position.z = 0;
    collision_path_point.pose.orientation = tf2::toMsg(quat);

    // obstacle position that can creates possible collision
    lanelet::BasicPoint2d obstacle_point = lanelet::geometry::fromArcCoordinates(
      path_lanelet.centerline2d(), {arc_lane_point_at_occlusion.length, signed_lateral_distance});
    ObstacleInfo obstacle_info;
    obstacle_info.position.x = obstacle_point[0];
    obstacle_info.position.y = obstacle_point[1];
    obstacle_info.position.z = 0;
    obstacle_info.max_velocity = param.pedestrian_vel;

    // collision point at front bumper
    lanelet::BasicPoint2d intersection_point = lanelet::geometry::fromArcCoordinates(
      path_lanelet.centerline2d(), {arc_lane_point_at_occlusion.length, 0.0});
    geometry_msgs::msg::Pose intersection_pose{};
    intersection_pose.position.x = intersection_point[0];
    intersection_pose.position.y = intersection_point[1];
    intersection_pose.position.z = 0;
    intersection_pose.orientation = tf2::toMsg(quat);

    possible_collisions.emplace_back(
      obstacle_info, collision_path_point, intersection_pose, arc_lane_dist_at_collision);
  }
}

void generatePossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions,
  const autoware_planning_msgs::msg::PathWithLaneId & path, const grid_map::GridMap & grid,
  int first_path_index, const PlannerParam & param, std::vector<lanelet::BasicPolygon2d> & debug)
{
  const double max_range = grid.getLength().maxCoeff() * 0.5;
  lanelet::ConstLanelet path_lanelet = buildPathLanelet(path, first_path_index, max_range);
  if (path_lanelet.centerline2d().empty()) {return;}
  // generate sidewalk possible collision
  generateSidewalkPossibleCollisions(possible_collisions, grid, path_lanelet, param, debug);
  possible_collisions.insert(
    possible_collisions.end(), possible_collisions.begin(), possible_collisions.end());
}

void generateSidewalkPossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param,
  std::vector<lanelet::BasicPolygon2d> & debug)
{
  std::vector<geometry::Slice> sidewalk_slices = geometry::buildSidewalkSlices(
    path_lanelet, param.vehicle_info.baselink_to_front * 0.5,
    param.vehicle_info.vehicle_width * 0.5, param.sidewalk.slice_size, param.sidewalk.focus_range);
  double length_lower_bound = std::numeric_limits<double>::max();
  double distance_lower_bound = std::numeric_limits<double>::max();
  for (const geometry::Slice & sidewalk_slice : sidewalk_slices) {
    debug.push_back(sidewalk_slice.polygon);
    if ((sidewalk_slice.range.min_length < length_lower_bound ||
      std::abs(sidewalk_slice.range.min_distance) < distance_lower_bound))
    {
      std::vector<grid_map::Position> occlusion_spot_positions;
      grid_utils::findOcclusionSpots(
        occlusion_spot_positions, grid, sidewalk_slice.polygon,
        param.sidewalk.min_occlusion_spot_size);
      generateSidewalkPossibleCollisionFromSlice(
        possible_collisions, grid, occlusion_spot_positions, path_lanelet, sidewalk_slice, param);
      if (!possible_collisions.empty()) {
        length_lower_bound = sidewalk_slice.range.min_length;
        distance_lower_bound = std::abs(sidewalk_slice.range.min_distance);
        possible_collisions.insert(
          possible_collisions.end(), possible_collisions.begin(), possible_collisions.end());
        debug.push_back(sidewalk_slice.polygon);
      }
    }
  }
}

void generateSidewalkPossibleCollisionFromSlice(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const std::vector<grid_map::Position> & occlusion_spot_positions,
  const lanelet::ConstLanelet & path_lanelet, const geometry::Slice & slice,
  const PlannerParam & param)
{
  const double half_vehicle_width = param.vehicle_info.vehicle_width * 0.5;
  for (grid_map::Position occlusion_spot_position : occlusion_spot_positions) {
    lanelet::ArcCoordinates occlusion_spot_ego_path_coordinates;
    occlusion_spot_ego_path_coordinates = {slice.range.min_length, slice.range.min_distance};
    // Assume an obstacle path orthogonal to the ego path
    const double ego_to_intersection_length = occlusion_spot_ego_path_coordinates.length;
    lanelet::BasicPoint2d intersection_point = lanelet::geometry::fromArcCoordinates(
      path_lanelet.centerline2d(), {ego_to_intersection_length, 0.0});
    bool collision_free =
      grid_utils::isCollisionFree(grid, occlusion_spot_position, intersection_point);
    if (collision_free) {
      const double ego_to_collision_length =
        ego_to_intersection_length - param.vehicle_info.baselink_to_front;
      const double ego_to_obstacle_distance =
        std::abs(occlusion_spot_ego_path_coordinates.distance) - half_vehicle_width;
      autoware_planning_msgs::msg::PathPoint collision_path_point;
      lanelet::BasicPoint2d col_point = lanelet::geometry::fromArcCoordinates(
        path_lanelet.centerline2d(), {ego_to_collision_length, 0.0});
      geometry_msgs::msg::Point search_point{};
      search_point.x = col_point[0];
      search_point.y = col_point[1];
      search_point.z = 0;
      double path_angle = lanelet::utils::getLaneletAngle(path_lanelet, search_point);
      tf2::Quaternion quat;
      quat.setRPY(0, 0, path_angle);
      collision_path_point.pose.position.x = col_point[0];
      collision_path_point.pose.position.y = col_point[1];
      collision_path_point.pose.orientation = tf2::toMsg(quat);
      ObstacleInfo obstacle_info;
      obstacle_info.position.x = occlusion_spot_position[0];
      obstacle_info.position.y = occlusion_spot_position[1];
      obstacle_info.max_velocity = param.pedestrian_vel;
      geometry_msgs::msg::Pose intersection_pose{};
      intersection_pose.position.x = intersection_point[0];
      intersection_pose.position.y = intersection_point[1];
      intersection_pose.orientation = tf2::toMsg(quat);
      lanelet::ArcCoordinates arc_lane_dist_at_collision;
      arc_lane_dist_at_collision.length = ego_to_collision_length;
      arc_lane_dist_at_collision.distance = ego_to_obstacle_distance;
      possible_collisions.emplace_back(
        obstacle_info, collision_path_point, intersection_pose, arc_lane_dist_at_collision);
      // if using slice coordinates, only need one occlusion_spot per slice
      break;
    }
  }
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
