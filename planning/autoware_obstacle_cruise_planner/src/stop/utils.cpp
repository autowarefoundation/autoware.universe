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

#include "autoware/obstacle_cruise_planner/stop/utils.hpp"

#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"

#include <boost/geometry.hpp>

#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace obstacle_cruise_utils
{
namespace
{
std::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPath(
  const PredictedPath & predicted_path, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time)
{
  const double rel_time = (current_time - obj_base_time).seconds();
  if (rel_time < 0.0) {
    return std::nullopt;
  }

  const auto pose =
    autoware::object_recognition_utils::calcInterpolatedPose(predicted_path, rel_time);
  if (!pose) {
    return std::nullopt;
  }
  return pose.get();
}

std::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPaths(
  const std::vector<PredictedPath> & predicted_paths, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time)
{
  if (predicted_paths.empty()) {
    return std::nullopt;
  }
  // Get the most reliable path
  const auto predicted_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  return getCurrentObjectPoseFromPredictedPath(*predicted_path, obj_base_time, current_time);
}
}  // namespace

std::vector<Polygon2d> createOneStepPolygons(
  const std::vector<TrajectoryPoint> & traj_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const CommonBehaviorDeterminationParam & common_behavior_determination_param)
{
  const auto & p = common_behavior_determination_param;

  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double vehicle_width = vehicle_info.vehicle_width_m;

  const size_t nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    autoware::universe_utils::inverseTransformPose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed{0.0};

  std::vector<Polygon2d> output_polygons;
  Polygon2d tmp_polys{};
  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};

    // estimate the future ego pose with assuming that the pose error against the reference path
    // will decrease to zero by the time_to_convergence
    if (p.enable_to_consider_current_pose && time_elapsed < p.time_to_convergence) {
      const double rem_ratio = (p.time_to_convergence - time_elapsed) / p.time_to_convergence;
      geometry_msgs::msg::Pose indexed_pose_err;
      indexed_pose_err.set__orientation(
        autoware::universe_utils::createQuaternionFromYaw(current_ego_yaw_error * rem_ratio));
      indexed_pose_err.set__position(
        autoware::universe_utils::createPoint(0.0, current_ego_lat_error * rem_ratio, 0.0));
      current_poses.push_back(
        autoware::universe_utils::transformPose(indexed_pose_err, traj_points.at(i).pose));
      if (traj_points.at(i).longitudinal_velocity_mps != 0.0) {
        time_elapsed +=
          p.decimate_trajectory_step_length / std::abs(traj_points.at(i).longitudinal_velocity_mps);
      } else {
        time_elapsed = std::numeric_limits<double>::max();
      }
    }

    Polygon2d idx_poly{};
    for (const auto & pose : current_poses) {
      if (i == 0 && traj_points.at(i).longitudinal_velocity_mps > 1e-3) {
        boost::geometry::append(
          idx_poly,
          autoware::universe_utils::toFootprint(pose, front_length, rear_length, vehicle_width)
            .outer());
        boost::geometry::append(
          idx_poly, autoware::universe_utils::fromMsg(
                      autoware::universe_utils::calcOffsetPose(
                        pose, front_length, vehicle_width * 0.5 + lat_margin, 0.0)
                        .position)
                      .to_2d());
        boost::geometry::append(
          idx_poly, autoware::universe_utils::fromMsg(
                      autoware::universe_utils::calcOffsetPose(
                        pose, front_length, -vehicle_width * 0.5 - lat_margin, 0.0)
                        .position)
                      .to_2d());
      } else {
        boost::geometry::append(
          idx_poly, autoware::universe_utils::toFootprint(
                      pose, front_length, rear_length, vehicle_width + lat_margin * 2.0)
                      .outer());
      }
    }

    boost::geometry::append(tmp_polys, idx_poly.outer());
    Polygon2d hull_polygon;
    boost::geometry::convex_hull(tmp_polys, hull_polygon);
    boost::geometry::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);
    tmp_polys = std::move(idx_poly);
  }
  return output_polygons;
}

std::vector<int> getTargetObjectType(rclcpp::Node & node, const std::string & param_prefix)
{
  std::unordered_map<std::string, int> types_map{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};

  std::vector<int> types;
  for (const auto & type : types_map) {
    if (node.declare_parameter<bool>(param_prefix + type.first)) {
      types.push_back(type.second);
    }
  }
  return types;
}

double calcObstacleMaxLength(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}
visualization_msgs::msg::Marker getObjectMarker(
  const geometry_msgs::msg::Pose & obj_pose, size_t idx, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();

  auto marker = autoware::universe_utils::createDefaultMarker(
    "map", current_time, ns, idx, visualization_msgs::msg::Marker::SPHERE,
    autoware::universe_utils::createMarkerScale(2.0, 2.0, 2.0),
    autoware::universe_utils::createMarkerColor(r, g, b, 0.8));

  marker.pose = obj_pose;

  return marker;
}

PoseWithStamp getCurrentObjectPose(
  const PredictedObject & predicted_object, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time, const bool use_prediction)
{
  const auto & pose = predicted_object.kinematics.initial_pose_with_covariance.pose;

  if (!use_prediction) {
    return PoseWithStamp{obj_base_time, pose};
  }

  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }
  const auto interpolated_pose =
    getCurrentObjectPoseFromPredictedPaths(predicted_paths, obj_base_time, current_time);

  if (!interpolated_pose) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner"), "Failed to find the interpolated obstacle pose");
    return PoseWithStamp{obj_base_time, pose};
  }

  return PoseWithStamp{obj_base_time, *interpolated_pose};
}

VelocityFactorArray makeVelocityFactorArray(
  const rclcpp::Time & time, const std::string & behavior,
  const std::optional<geometry_msgs::msg::Pose> pose)
{
  VelocityFactorArray velocity_factor_array;
  velocity_factor_array.header.frame_id = "map";
  velocity_factor_array.header.stamp = time;

  if (pose) {
    using distance_type = VelocityFactor::_distance_type;
    VelocityFactor velocity_factor;
    velocity_factor.behavior = behavior;
    velocity_factor.pose = pose.value();
    velocity_factor.distance = std::numeric_limits<distance_type>::quiet_NaN();
    velocity_factor.status = VelocityFactor::UNKNOWN;
    velocity_factor.detail = std::string();
    velocity_factor_array.factors.push_back(velocity_factor);
  }
  return velocity_factor_array;
}

}  // namespace obstacle_cruise_utils
