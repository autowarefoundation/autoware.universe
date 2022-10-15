// Copyright 2022 Tier IV, Inc.
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

#include "collision_free_path_planner/debug_marker.hpp"

#include "collision_free_path_planner/eb_path_optimizer.hpp"
#include "collision_free_path_planner/mpt_optimizer.hpp"
#include "collision_free_path_planner/utils/cv_utils.hpp"
// #include "collision_free_path_planner/utils/utils.hpp"
#include "motion_utils/motion_utils.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

namespace collision_free_path_planner
{
namespace
{
template <typename T>
MarkerArray getPointsMarkerArray(
  const std::vector<T> & points, const std::string & ns, const double r, const double g,
  const double b)
{
  if (points.empty()) {
    return MarkerArray{};
  }

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, Marker::LINE_LIST, createMarkerScale(0.5, 0.5, 0.5),
    createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  for (const auto & point : points) {
    marker.points.push_back(tier4_autoware_utils::getPoint(point));
  }

  MarkerArray msg;
  msg.markers.push_back(marker);

  return msg;
}

template <typename T>
MarkerArray getPointsTextMarkerArray(
  const std::vector<T> & points, const std::string & ns, const double r, const double g,
  const double b)
{
  if (points.empty()) {
    return MarkerArray{};
  }

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.0, 0.0, 0.15), createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  MarkerArray msg;
  for (size_t i = 0; i < points.size(); i++) {
    marker.id = i;
    marker.text = std::to_string(i);
    marker.pose.position = tier4_autoware_utils::getPoint(points[i]);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray getDebugConstrainMarkers(
  const std::vector<ConstrainRectangle> & constrain_ranges, const std::string & ns)
{
  MarkerArray marker_array;
  int unique_id = 0;
  for (size_t i = 0; i < constrain_ranges.size(); i++) {
    Marker constrain_rect_marker;
    constrain_rect_marker.lifetime = rclcpp::Duration::from_seconds(0);
    constrain_rect_marker.header.frame_id = "map";
    constrain_rect_marker.header.stamp = rclcpp::Time(0);
    constrain_rect_marker.ns = ns;
    constrain_rect_marker.action = Marker::ADD;
    constrain_rect_marker.pose.orientation.w = 1.0;
    constrain_rect_marker.id = unique_id;
    constrain_rect_marker.type = Marker::LINE_STRIP;
    constrain_rect_marker.scale = createMarkerScale(0.01, 0, 0);
    constrain_rect_marker.color = createMarkerColor(1.0, 0, 0, 0.99);
    unique_id++;
    geometry_msgs::msg::Point top_left_point = constrain_ranges[i].top_left;
    geometry_msgs::msg::Point top_right_point = constrain_ranges[i].top_right;
    geometry_msgs::msg::Point bottom_right_point = constrain_ranges[i].bottom_right;
    geometry_msgs::msg::Point bottom_left_point = constrain_ranges[i].bottom_left;
    constrain_rect_marker.points.push_back(top_left_point);
    constrain_rect_marker.points.push_back(top_right_point);
    constrain_rect_marker.points.push_back(bottom_right_point);
    constrain_rect_marker.points.push_back(bottom_left_point);
    constrain_rect_marker.points.push_back(top_left_point);
    marker_array.markers.push_back(constrain_rect_marker);
  }

  for (size_t i = 0; i < constrain_ranges.size(); i++) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = ns + "_text";
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.scale = createMarkerScale(0, 0, 0.15);
    marker.color = createMarkerColor(1.0, 0, 0, 0.99);
    marker.text = std::to_string(i);
    marker.pose.position = constrain_ranges[i].top_left;
    marker_array.markers.push_back(marker);
  }

  unique_id = 0;
  for (size_t i = 0; i < constrain_ranges.size(); i++) {
    Marker constrain_range_text_marker;
    constrain_range_text_marker.lifetime = rclcpp::Duration::from_seconds(0);
    constrain_range_text_marker.header.frame_id = "map";
    constrain_range_text_marker.header.stamp = rclcpp::Time(0);
    constrain_range_text_marker.ns = ns + "_location";
    constrain_range_text_marker.action = Marker::ADD;
    constrain_range_text_marker.pose.orientation.w = 1.0;
    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.type = Marker::TEXT_VIEW_FACING;
    constrain_range_text_marker.pose.position = constrain_ranges[i].top_left;
    constrain_range_text_marker.scale = createMarkerScale(0, 0, 0.1);
    constrain_range_text_marker.color = createMarkerColor(1.0, 0, 0, 0.99);
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].top_right;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].bottom_left;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].bottom_right;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);
  }
  return marker_array;
}

MarkerArray getObjectsMarkerArray(
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const std::string & ns, const double r, const double g, const double b)
{
  MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, Marker::CUBE, createMarkerScale(3.0, 1.0, 1.0),
    createMarkerColor(r, g, b, 0.8));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t i = 0; i < objects.size(); ++i) {
    const auto & object = objects.at(i);
    marker.id = i;
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray getRectanglesMarkerArray(
  const std::vector<TrajectoryPoint> & mpt_traj,
  const vehicle_info_util::VehicleInfo & vehicle_info, const std::string & ns, const double r,
  const double g, const double b, const size_t sampling_num)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, Marker::LINE_STRIP, createMarkerScale(0.05, 0.0, 0.0),
    createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  MarkerArray marker_array;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }

    marker.id = i;
    marker.points.clear();

    const auto & traj_point = mpt_traj.at(i);

    const double half_width = vehicle_info.vehicle_width_m / 2.0;
    const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
    const double base_to_rear = vehicle_info.rear_overhang_m;

    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, base_to_front, -half_width, 0.0)
        .position);
    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, base_to_front, half_width, 0.0)
        .position);
    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, -base_to_rear, half_width, 0.0)
        .position);
    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, -base_to_rear, -half_width, 0.0)
        .position);
    marker.points.push_back(marker.points.front());

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray getRectanglesNumMarkerArray(
  const std::vector<TrajectoryPoint> mpt_traj, const vehicle_info_util::VehicleInfo & vehicle_info,
  const std::string & ns, const double r, const double g, const double b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.0, 0.0, 0.125), createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  MarkerArray msg;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    const auto & traj_point = mpt_traj.at(i);

    marker.text = std::to_string(i);

    const double half_width = vehicle_info.vehicle_width_m / 2.0;
    const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;

    const auto top_right_pos =
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, base_to_front, half_width, 0.0)
        .position;
    marker.id = i;
    marker.pose.position = top_right_pos;
    msg.markers.push_back(marker);

    marker.id = i + mpt_traj.size();
    marker.pose.position = top_right_pos;
    msg.markers.push_back(marker);
  }
  return msg;
}

MarkerArray getBoundsCandidatesLineMarkerArray(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<std::vector<Bounds>> & bounds_candidates, const double r, const double g,
  const double b, [[maybe_unused]] const double vehicle_width, const size_t sampling_num)
{
  const auto current_time = rclcpp::Clock().now();
  MarkerArray msg;
  const std::string ns = "bounds_candidates";

  if (ref_points.empty()) return msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, Marker::LINE_LIST, createMarkerScale(0.05, 0.0, 0.0),
    createMarkerColor(r + 0.5, g, b, 0.3));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t i = 0; i < ref_points.size(); i++) {
    if (i % sampling_num != 0) {
      continue;
    }

    const auto & bound_candidates = bounds_candidates.at(i);
    for (size_t j = 0; j < bound_candidates.size(); ++j) {
      geometry_msgs::msg::Pose pose;
      pose.position = ref_points.at(i).p;
      pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(ref_points.at(i).yaw);

      // lower bound
      const double lb_y = bound_candidates.at(j).lower_bound;
      const auto lb = tier4_autoware_utils::calcOffsetPose(pose, 0.0, lb_y, 0.0).position;
      marker.points.push_back(lb);

      // upper bound
      const double ub_y = bound_candidates.at(j).upper_bound;
      const auto ub = tier4_autoware_utils::calcOffsetPose(pose, 0.0, ub_y, 0.0).position;
      marker.points.push_back(ub);

      msg.markers.push_back(marker);
    }
  }

  return msg;
}

MarkerArray getBoundsLineMarkerArray(
  const std::vector<ReferencePoint> & ref_points, const double r, const double g, const double b,
  const double vehicle_width, const size_t sampling_num)
{
  const auto current_time = rclcpp::Clock().now();
  MarkerArray marker_array;

  if (ref_points.empty()) return marker_array;

  // craete lower bound marker
  auto lb_marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), "", 0, Marker::LINE_LIST, createMarkerScale(0.05, 0.0, 0.0),
    createMarkerColor(r + 0.5, g, b, 0.3));
  lb_marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  // craete upper bound marker
  auto ub_marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), "", 1, Marker::LINE_LIST, createMarkerScale(0.05, 0.0, 0.0),
    createMarkerColor(r, g + 0.5, b, 0.3));
  ub_marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t bound_idx = 0; bound_idx < ref_points.at(0).vehicle_bounds.size(); ++bound_idx) {
    const std::string ns = "base_bounds_" + std::to_string(bound_idx);

    {  // lower bound
      lb_marker.points.clear();
      lb_marker.ns = ns;

      for (size_t i = 0; i < ref_points.size(); i++) {
        if (i % sampling_num != 0) {
          continue;
        }

        const geometry_msgs::msg::Pose & pose = ref_points.at(i).vehicle_bounds_poses.at(bound_idx);
        const double lb_y =
          ref_points.at(i).vehicle_bounds[bound_idx].lower_bound - vehicle_width / 2.0;
        const auto lb = tier4_autoware_utils::calcOffsetPose(pose, 0.0, lb_y, 0.0).position;

        lb_marker.points.push_back(pose.position);
        lb_marker.points.push_back(lb);
      }
      marker_array.markers.push_back(lb_marker);
    }

    {  // upper bound
      ub_marker.points.clear();
      ub_marker.ns = ns;

      for (size_t i = 0; i < ref_points.size(); i++) {
        if (i % sampling_num != 0) {
          continue;
        }

        const geometry_msgs::msg::Pose & pose = ref_points.at(i).vehicle_bounds_poses.at(bound_idx);
        const double ub_y =
          ref_points.at(i).vehicle_bounds[bound_idx].upper_bound + vehicle_width / 2.0;
        const auto ub = tier4_autoware_utils::calcOffsetPose(pose, 0.0, ub_y, 0.0).position;

        ub_marker.points.push_back(pose.position);
        ub_marker.points.push_back(ub);
      }
      marker_array.markers.push_back(ub_marker);
    }
  }

  return marker_array;
}

MarkerArray getVehicleCircleLineMarkerArray(
  const std::vector<std::vector<geometry_msgs::msg::Pose>> & vehicle_circles_pose,
  const double vehicle_width, const size_t sampling_num, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  MarkerArray msg;

  for (size_t i = 0; i < vehicle_circles_pose.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }

    auto marker = createDefaultMarker(
      "map", rclcpp::Clock().now(), ns, i, Marker::LINE_LIST, createMarkerScale(0.1, 0, 0),
      createMarkerColor(r, g, b, 0.25));
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);

    for (size_t j = 0; j < vehicle_circles_pose.at(i).size(); ++j) {
      const geometry_msgs::msg::Pose & pose = vehicle_circles_pose.at(i).at(j);
      const auto ub =
        tier4_autoware_utils::calcOffsetPose(pose, 0.0, vehicle_width / 2.0, 0.0).position;
      const auto lb =
        tier4_autoware_utils::calcOffsetPose(pose, 0.0, -vehicle_width / 2.0, 0.0).position;

      marker.points.push_back(ub);
      marker.points.push_back(lb);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray getLateralErrorsLineMarkerArray(
  const std::vector<geometry_msgs::msg::Pose> mpt_ref_poses, std::vector<double> lateral_errors,
  const size_t sampling_num, const std::string & ns, const double r, const double g, const double b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, Marker::LINE_LIST, createMarkerScale(0.1, 0, 0),
    createMarkerColor(r, g, b, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t i = 0; i < mpt_ref_poses.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }

    const auto vehicle_pose =
      tier4_autoware_utils::calcOffsetPose(mpt_ref_poses.at(i), 0.0, lateral_errors.at(i), 0.0);
    marker.points.push_back(mpt_ref_poses.at(i).position);
    marker.points.push_back(vehicle_pose.position);
  }

  MarkerArray msg;
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray getCurrentVehicleCirclesMarkerArray(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<double> & vehicle_circle_longitudinal_offsets,
  const std::vector<double> & vehicle_circle_radiuses, const std::string & ns, const double r,
  const double g, const double b)
{
  MarkerArray msg;

  size_t id = 0;
  for (size_t v_idx = 0; v_idx < vehicle_circle_longitudinal_offsets.size(); ++v_idx) {
    const double offset = vehicle_circle_longitudinal_offsets.at(v_idx);

    auto marker = createDefaultMarker(
      "map", rclcpp::Clock().now(), ns, id, Marker::LINE_STRIP, createMarkerScale(0.05, 0.0, 0.0),
      createMarkerColor(r, g, b, 0.8));
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    marker.pose = tier4_autoware_utils::calcOffsetPose(ego_pose, offset, 0.0, 0.0);

    constexpr size_t circle_dividing_num = 16;
    for (size_t e_idx = 0; e_idx < circle_dividing_num + 1; ++e_idx) {
      geometry_msgs::msg::Point edge_pos;

      const double edge_angle =
        static_cast<double>(e_idx) / static_cast<double>(circle_dividing_num) * 2.0 * M_PI;
      edge_pos.x = vehicle_circle_radiuses.at(v_idx) * std::cos(edge_angle);
      edge_pos.y = vehicle_circle_radiuses.at(v_idx) * std::sin(edge_angle);

      marker.points.push_back(edge_pos);
    }

    msg.markers.push_back(marker);
    ++id;
  }
  return msg;
}

MarkerArray getVehicleCirclesMarkerArray(
  const std::vector<TrajectoryPoint> & mpt_traj_points,
  const std::vector<double> & vehicle_circle_longitudinal_offsets,
  const std::vector<double> & vehicle_circle_radiuses, const size_t sampling_num,
  const std::string & ns, const double r, const double g, const double b)
{
  MarkerArray msg;

  size_t id = 0;
  for (size_t i = 0; i < mpt_traj_points.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }
    const auto & mpt_traj_point = mpt_traj_points.at(i);

    for (size_t v_idx = 0; v_idx < vehicle_circle_longitudinal_offsets.size(); ++v_idx) {
      const double offset = vehicle_circle_longitudinal_offsets.at(v_idx);

      auto marker = createDefaultMarker(
        "map", rclcpp::Clock().now(), ns, id, Marker::LINE_STRIP, createMarkerScale(0.05, 0.0, 0.0),
        createMarkerColor(r, g, b, 0.8));
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);
      marker.pose = tier4_autoware_utils::calcOffsetPose(mpt_traj_point.pose, offset, 0.0, 0.0);

      constexpr size_t circle_dividing_num = 16;
      for (size_t e_idx = 0; e_idx < circle_dividing_num + 1; ++e_idx) {
        geometry_msgs::msg::Point edge_pos;

        const double edge_angle =
          static_cast<double>(e_idx) / static_cast<double>(circle_dividing_num) * 2.0 * M_PI;
        edge_pos.x = vehicle_circle_radiuses.at(v_idx) * std::cos(edge_angle);
        edge_pos.y = vehicle_circle_radiuses.at(v_idx) * std::sin(edge_angle);

        marker.points.push_back(edge_pos);
      }

      msg.markers.push_back(marker);
      ++id;
    }
  }
  return msg;
}
}  // namespace

MarkerArray getDebugMarker(
  const DebugData & debug_data, const std::vector<TrajectoryPoint> & optimized_points,
  const vehicle_info_util::VehicleInfo & vehicle_info, const bool is_showing_debug_detail)
{
  MarkerArray marker_array;

  if (is_showing_debug_detail) {
    appendMarkerArray(
      getDebugConstrainMarkers(debug_data.constrain_rectangles, "constrain_rect"), &marker_array);

    appendMarkerArray(
      getRectanglesNumMarkerArray(
        optimized_points, vehicle_info, "num_vehicle_footprint", 0.99, 0.99, 0.2),
      &marker_array);

    appendMarkerArray(
      getPointsTextMarkerArray(debug_data.eb_traj, "eb_traj_text", 0.99, 0.99, 0.2), &marker_array);

    // avoiding objects
    appendMarkerArray(
      getObjectsMarkerArray(debug_data.avoiding_objects, "avoiding_objects", 0.99, 0.99, 0.2),
      &marker_array);

    // lateral error line
    appendMarkerArray(
      getLateralErrorsLineMarkerArray(
        debug_data.mpt_ref_poses, debug_data.lateral_errors, debug_data.mpt_visualize_sampling_num,
        "lateral_errors", 0.1, 0.1, 0.8),
      &marker_array);
  }

  // mpt footprints
  appendMarkerArray(
    getRectanglesMarkerArray(
      optimized_points, vehicle_info, "mpt_footprints", 0.99, 0.99, 0.2,
      debug_data.mpt_visualize_sampling_num),
    &marker_array);

  // bounds
  appendMarkerArray(
    getBoundsLineMarkerArray(
      debug_data.ref_points, 0.99, 0.99, 0.2, vehicle_info.vehicle_width_m,
      debug_data.mpt_visualize_sampling_num),
    &marker_array);

  // bounds candidates
  appendMarkerArray(
    getBoundsCandidatesLineMarkerArray(
      debug_data.ref_points, debug_data.sequential_bounds_candidates, 0.2, 0.99, 0.99,
      vehicle_info.vehicle_width_m, debug_data.mpt_visualize_sampling_num),
    &marker_array);

  // vehicle circle line
  appendMarkerArray(
    getVehicleCircleLineMarkerArray(
      debug_data.vehicle_circles_pose, vehicle_info.vehicle_width_m,
      debug_data.mpt_visualize_sampling_num, "vehicle_circle_lines", 0.99, 0.99, 0.2),
    &marker_array);

  // current vehicle circles
  appendMarkerArray(
    getCurrentVehicleCirclesMarkerArray(
      debug_data.ego_pose, debug_data.vehicle_circle_longitudinal_offsets,
      debug_data.vehicle_circle_radiuses, "current_vehicle_circles", 1.0, 0.3, 0.3),
    &marker_array);

  // vehicle circles
  appendMarkerArray(
    getVehicleCirclesMarkerArray(
      optimized_points, debug_data.vehicle_circle_longitudinal_offsets,
      debug_data.vehicle_circle_radiuses, debug_data.mpt_visualize_sampling_num, "vehicle_circles",
      1.0, 0.3, 0.3),
    &marker_array);

  return marker_array;
}

OccupancyGrid getDebugCostmap(const cv::Mat & clearance_map, const OccupancyGrid & occupancy_grid)
{
  if (clearance_map.empty()) return OccupancyGrid();

  cv::Mat tmp;
  clearance_map.copyTo(tmp);
  cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  OccupancyGrid clearance_map_in_og = occupancy_grid;
  tmp.forEach<unsigned char>([&](const unsigned char & value, const int * position) -> void {
    cv_utils::putOccupancyGridValue(clearance_map_in_og, position[0], position[1], value);
  });
  return clearance_map_in_og;
}
}  // namespace collision_free_path_planner
