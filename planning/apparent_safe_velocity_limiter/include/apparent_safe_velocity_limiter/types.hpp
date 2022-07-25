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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__TYPES_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__TYPES_HPP_

#include <rclcpp/node.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace apparent_safe_velocity_limiter
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::OccupancyGrid;
using PointCloud = sensor_msgs::msg::PointCloud2;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using Float = decltype(TrajectoryPoint::longitudinal_velocity_mps);

using point_t = tier4_autoware_utils::Point2d;
using polygon_t = tier4_autoware_utils::Polygon2d;
using multipolygon_t = tier4_autoware_utils::MultiPolygon2d;
using segment_t = tier4_autoware_utils::Segment2d;
using linestring_t = tier4_autoware_utils::LineString2d;
using multilinestring_t = tier4_autoware_utils::MultiLineString2d;

struct ObstacleParameters
{
  static constexpr auto DYN_SOURCE_PARAM = "obstacles.dynamic_source";
  static constexpr auto OCC_GRID_THRESH_PARAM = "obstacles.occupancy_grid_threshold";
  static constexpr auto BUFFER_PARAM = "obstacles.dynamic_obstacles_buffer";
  static constexpr auto MIN_VEL_PARAM = "obstacles.dynamic_obstacles_min_vel";
  static constexpr auto MAP_TAGS_PARAM = "obstacles.static_map_tags";
  static constexpr auto FILTERING_PARAM = "obstacles.filter_envelope";

  enum { POINTCLOUD, OCCUPANCYGRID, STATIC_ONLY } dynamic_source = OCCUPANCYGRID;
  int8_t occupancy_grid_threshold{};
  Float pcd_cluster_max_dist{};
  Float dynamic_obstacles_buffer{};
  Float dynamic_obstacles_min_vel{};
  std::vector<std::string> static_map_tags{};
  bool filter_envelope;

  ObstacleParameters() = default;
  explicit ObstacleParameters(rclcpp::Node & node)
  {
    updateType(node, node.declare_parameter<std::string>(DYN_SOURCE_PARAM));
    occupancy_grid_threshold =
      static_cast<int8_t>(node.declare_parameter<int>(OCC_GRID_THRESH_PARAM));
    dynamic_obstacles_buffer = static_cast<Float>(node.declare_parameter<Float>(BUFFER_PARAM));
    dynamic_obstacles_min_vel = static_cast<Float>(node.declare_parameter<Float>(MIN_VEL_PARAM));
    static_map_tags = node.declare_parameter<std::vector<std::string>>(MAP_TAGS_PARAM);
    filter_envelope = node.declare_parameter<bool>(FILTERING_PARAM);
  }

  bool updateType(rclcpp::Node & node, const std::string & type)
  {
    if (type == "pointcloud") {
      dynamic_source = POINTCLOUD;
    } else if (type == "occupancy_grid") {
      dynamic_source = OCCUPANCYGRID;
    } else if (type == "static_only") {
      dynamic_source = STATIC_ONLY;
    } else {
      dynamic_source = STATIC_ONLY;
      RCLCPP_WARN(
        node.get_logger(), "Unknown '%s' value: '%s'. Using default 'static_only'.",
        DYN_SOURCE_PARAM, type.c_str());
      return false;
    }
    return true;
  }
};

struct ProjectionParameters
{
  static constexpr auto MODEL_PARAM = "forward_projection.model";
  static constexpr auto NBPOINTS_PARAM = "forward_projection.nb_points";
  static constexpr auto STEER_OFFSET_PARAM = "forward_projection.steering_offset";
  static constexpr auto DISTANCE_METHOD_PARAM = "forward_projection.distance_method";
  static constexpr auto DURATION_PARAM = "min_ttc";

  enum { PARTICLE, BICYCLE } model = PARTICLE;
  enum { EXACT, APPROXIMATION } distance_method = EXACT;
  double duration{};
  double extra_length{};
  double velocity{};
  double heading{};
  // parameters specific to the bicycle model
  int points_per_projection = 5;
  double wheel_base{};
  double steering_angle{};
  double steering_angle_offset{};

  ProjectionParameters() = default;
  explicit ProjectionParameters(rclcpp::Node & node)
  {
    updateModel(node, node.declare_parameter<std::string>(MODEL_PARAM));
    updateDistanceMethod(node, node.declare_parameter<std::string>(DISTANCE_METHOD_PARAM));
    updateNbPoints(node, node.declare_parameter<int>(NBPOINTS_PARAM));
    steering_angle_offset = node.declare_parameter<double>(STEER_OFFSET_PARAM);
    duration = node.declare_parameter<double>(DURATION_PARAM);
  }

  bool updateModel(rclcpp::Node & node, const std::string & model_str)
  {
    if (model_str == "particle") {
      model = PARTICLE;
    } else if (model_str == "bicycle") {
      model = BICYCLE;
    } else {
      RCLCPP_WARN(
        node.get_logger(), "Unknown projection model: '%s'. Using default PARTICLE model.",
        model_str.c_str());
      return false;
    }
    return true;
  }

  bool updateDistanceMethod(rclcpp::Node & node, const std::string & method_str)
  {
    if (method_str == "exact") {
      distance_method = EXACT;
    } else if (method_str == "approximation") {
      distance_method = APPROXIMATION;
    } else {
      RCLCPP_WARN(
        node.get_logger(), "Unknown distance calculation method: '%s'. Using default EXACT method.",
        method_str.c_str());
      return false;
    }
    return true;
  }

  bool updateNbPoints(rclcpp::Node & node, const int nb_points)
  {
    if (nb_points < 2) {
      RCLCPP_WARN(
        node.get_logger(), "Cannot use less than 2 points per projection. Using value %d instead.",
        points_per_projection);
      return false;
    }
    points_per_projection = nb_points;
    return true;
  }

  void update(const TrajectoryPoint & point)
  {
    velocity = point.longitudinal_velocity_mps;
    heading = tf2::getYaw(point.pose.orientation);
    steering_angle = point.front_wheel_angle_rad;
  }
};

struct VelocityParameters
{
  static constexpr auto MIN_VEL_PARAM = "min_adjusted_velocity";
  static constexpr auto MAX_DECEL_PARAM = "max_deceleration";

  Float min_velocity{};
  Float max_deceleration{};
  Float current_ego_velocity{};

  VelocityParameters() = default;
  explicit VelocityParameters(rclcpp::Node & node)
  {
    min_velocity = static_cast<Float>(node.declare_parameter<double>(MIN_VEL_PARAM));
    max_deceleration = static_cast<Float>(node.declare_parameter<double>(MAX_DECEL_PARAM));
  }
};

}  // namespace apparent_safe_velocity_limiter
#endif  // APPARENT_SAFE_VELOCITY_LIMITER__TYPES_HPP_
