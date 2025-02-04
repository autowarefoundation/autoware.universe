// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "autoware/obstacle_collision_checker/obstacle_collision_checker.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#include <vector>

namespace
{
pcl::PointCloud<pcl::PointXYZ> get_transformed_point_cloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg,
  const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}

pcl::PointCloud<pcl::PointXYZ> filter_point_cloud_by_trajectory(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double radius)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
  for (const auto & point : pointcloud.points) {
    for (const auto & trajectory_point : trajectory.points) {
      const double dx = trajectory_point.pose.position.x - point.x;
      const double dy = trajectory_point.pose.position.y - point.y;
      if (std::hypot(dx, dy) < radius) {
        filtered_pointcloud.points.push_back(point);
        break;
      }
    }
  }
  return filtered_pointcloud;
}

double calc_braking_distance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  const double idling_distance = abs_velocity * delay_time;
  const double braking_distance = (abs_velocity * abs_velocity) / (2.0 * max_deceleration);
  return idling_distance + braking_distance;
}

}  // namespace

namespace autoware::obstacle_collision_checker
{
Output check_for_collisions(const Input & input)
{
  Output output;
  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  // resample trajectory by braking distance
  constexpr double min_velocity = 0.01;
  const auto & raw_abs_velocity = std::abs(input.current_twist->linear.x);
  const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;
  const auto braking_distance =
    calc_braking_distance(abs_velocity, input.param.max_deceleration, input.param.delay_time);
  output.resampled_trajectory = cut_trajectory(
    resample_trajectory(*input.predicted_trajectory, input.param.resample_interval),
    braking_distance);
  output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);

  // resample pointcloud
  const auto obstacle_pointcloud =
    get_transformed_point_cloud(*input.obstacle_pointcloud, input.obstacle_transform->transform);
  const auto filtered_obstacle_pointcloud = filter_point_cloud_by_trajectory(
    obstacle_pointcloud, output.resampled_trajectory, input.param.search_radius);

  output.vehicle_footprints =
    create_vehicle_footprints(output.resampled_trajectory, input.param, input.vehicle_info);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = create_vehicle_passing_areas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  output.will_collide = will_collide(filtered_obstacle_pointcloud, output.vehicle_passing_areas);
  output.processing_time_map["willCollide"] = stop_watch.toc(true);

  return output;
}

autoware_planning_msgs::msg::Trajectory resample_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double interval)
{
  autoware_planning_msgs::msg::Trajectory resampled;
  resampled.header = trajectory.header;

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & point = trajectory.points.at(i);

    const auto distance =
      autoware::universe_utils::calcDistance2d(resampled.points.back(), point.pose.position);
    if (distance > interval) {
      resampled.points.push_back(point);
    }
  }
  resampled.points.push_back(trajectory.points.back());

  return resampled;
}

autoware_planning_msgs::msg::Trajectory cut_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double length)
{
  autoware_planning_msgs::msg::Trajectory cut;
  cut.header = trajectory.header;

  double total_length = 0.0;
  cut.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware::universe_utils::fromMsg(cut.points.back().pose.position);
    const auto p2 = autoware::universe_utils::fromMsg(point.pose.position);

    const auto points_distance = boost::geometry::distance(p1, p2);
    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0.0) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      autoware_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.points.push_back(p);
      break;
    }

    cut.points.push_back(point);
    total_length += points_distance;
  }

  return cut;
}

std::vector<LinearRing2d> create_vehicle_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const Param & param,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(param.footprint_margin);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory.points) {
    vehicle_footprints.push_back(
      autoware::universe_utils::transformVector<autoware::universe_utils::LinearRing2d>(
        local_vehicle_footprint, autoware::universe_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> create_vehicle_passing_areas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  // Create hull from two adjacent vehicle footprints
  std::vector<LinearRing2d> areas;
  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(create_hull_from_footprints(footprint1, footprint2));
  }

  return areas;
}

LinearRing2d create_hull_from_footprints(const LinearRing2d & area1, const LinearRing2d & area2)
{
  autoware::universe_utils::MultiPoint2d combined;
  for (const auto & p : area1) {
    combined.push_back(p);
  }
  for (const auto & p : area2) {
    combined.push_back(p);
  }
  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);
  return hull;
}

bool will_collide(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  for (size_t i = 1; i < vehicle_footprints.size(); i++) {
    // skip first footprint because surround obstacle checker handle it
    const auto & vehicle_footprint = vehicle_footprints.at(i);
    if (has_collision(obstacle_pointcloud, vehicle_footprint)) {
      RCLCPP_WARN(rclcpp::get_logger("obstacle_collision_checker"), "willCollide");
      return true;
    }
  }

  return false;
}

bool has_collision(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : obstacle_pointcloud.points) {
    if (boost::geometry::within(
          autoware::universe_utils::Point2d{point.x, point.y}, vehicle_footprint)) {
      RCLCPP_WARN(
        rclcpp::get_logger("obstacle_collision_checker"), "Collide to Point x: %f y: %f", point.x,
        point.y);
      return true;
    }
  }

  return false;
}
}  // namespace autoware::obstacle_collision_checker
