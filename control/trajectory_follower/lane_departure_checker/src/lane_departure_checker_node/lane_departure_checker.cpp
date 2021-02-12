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

#include <algorithm>
#include <vector>

#include "lane_departure_checker/lane_departure_checker.hpp"

#include "boost/geometry.hpp"

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/normalization.hpp"
#include "autoware_utils/math/unit_conversion.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "lanelet2_core/geometry/Polygon.h"
#include "tf2/utils.h"

#include "lane_departure_checker/util/create_vehicle_footprint.hpp"

using autoware_utils::LinearRing2d;
using autoware_utils::MultiPoint2d;
using autoware_utils::Point2d;

namespace
{
double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity;
}

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  for (const auto & ll : candidate_lanelets) {
    if (boost::geometry::within(point, ll.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

size_t findNearestIndex(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose)
{
  std::vector<double> distances;
  distances.reserve(trajectory.points.size());
  std::transform(
    trajectory.points.cbegin(), trajectory.points.cend(), std::back_inserter(distances),
    [&](const autoware_planning_msgs::msg::TrajectoryPoint & p) {
      const auto p1 = autoware_utils::fromMsg(p.pose.position).to_2d();
      const auto p2 = autoware_utils::fromMsg(pose.position).to_2d();
      return boost::geometry::distance(p1, p2);
    });

  const auto min_itr = std::min_element(distances.cbegin(), distances.cend());
  const auto min_idx = static_cast<size_t>(std::distance(distances.cbegin(), min_itr));

  return min_idx;
}

boost::optional<lanelet::ConstLanelet> findNearestLanelet(
  const lanelet::ConstLanelets & lanelets, const Point2d & point, const double th_dist)
{
  std::vector<double> distances;
  distances.reserve(lanelets.size());
  std::transform(
    lanelets.cbegin(), lanelets.cend(), std::back_inserter(distances),
    [&](const lanelet::ConstLanelet & ll) {
      return boost::geometry::distance(ll.polygon2d().basicPolygon(), point);
    });

  const auto min_itr = std::min_element(distances.cbegin(), distances.cend());
  const auto min_idx = static_cast<size_t>(std::distance(distances.cbegin(), min_itr));

  if (*min_itr > th_dist) {
    return {};
  }

  return lanelets.at(min_idx);
}

lanelet::ConstLanelets getCandidateLanelets(
  const lanelet::ConstLanelets & route_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  lanelet::ConstLanelets candidate_lanelets;

  for (const auto & vehicle_footprint : vehicle_footprints) {
    for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
      const auto & p = vehicle_footprint.at(i);

      const auto nearest_lanelet = findNearestLanelet(route_lanelets, p, 10.0);
      if (nearest_lanelet) {
        candidate_lanelets.push_back(*nearest_lanelet);
      }
    }
  }

  return candidate_lanelets;
}
}  // namespace

namespace lane_departure_checker
{
Output LaneDepartureChecker::update(const Input & input)
{
  Output output{};

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  output.trajectory_deviation =
    calcTrajectoryDeviation(*input.reference_trajectory, input.current_pose->pose);
  output.processing_time_map["calcTrajectoryDeviation"] = stop_watch.toc(true);

  {
    constexpr double min_velocity = 0.01;
    const auto & raw_abs_velocity = std::abs(input.current_twist->twist.linear.x);
    const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;

    const auto braking_distance =
      calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time);

    output.resampled_trajectory = cutTrajectory(
      resampleTrajectory(*input.predicted_trajectory, param_.resample_interval), braking_distance);
    output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);
  }

  output.vehicle_footprints = createVehicleFootprints(output.resampled_trajectory, param_);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = createVehiclePassingAreas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  output.candidate_lanelets = getCandidateLanelets(input.route_lanelets, output.vehicle_footprints);
  output.processing_time_map["getCandidateLanelets"] = stop_watch.toc(true);

  output.will_leave_lane = willLeaveLane(output.candidate_lanelets, output.vehicle_footprints);
  output.processing_time_map["willLeaveLane"] = stop_watch.toc(true);

  output.is_out_of_lane = isOutOfLane(output.candidate_lanelets, output.vehicle_footprints.front());
  output.processing_time_map["isOutOfLane"] = stop_watch.toc(true);

  return output;
}

PoseDeviation LaneDepartureChecker::calcTrajectoryDeviation(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose)
{
  const auto nearest_idx = findNearestIndex(trajectory, pose);
  return autoware_utils::calcPoseDeviation(trajectory.points.at(nearest_idx).pose, pose);
}

autoware_planning_msgs::msg::Trajectory LaneDepartureChecker::resampleTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double interval)
{
  autoware_planning_msgs::msg::Trajectory resampled;
  resampled.header = trajectory.header;

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware_utils::fromMsg(resampled.points.back().pose.position);
    const auto p2 = autoware_utils::fromMsg(point.pose.position);

    if (boost::geometry::distance(p1.to_2d(), p2.to_2d()) > interval) {
      resampled.points.push_back(point);
    }
  }
  resampled.points.push_back(trajectory.points.back());

  return resampled;
}

autoware_planning_msgs::msg::Trajectory LaneDepartureChecker::cutTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double length)
{
  autoware_planning_msgs::msg::Trajectory cut;
  cut.header = trajectory.header;

  double total_length = 0.0;
  cut.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware_utils::fromMsg(cut.points.back().pose.position);
    const auto p2 = autoware_utils::fromMsg(point.pose.position);
    const auto points_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
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

std::vector<LinearRing2d> LaneDepartureChecker::createVehicleFootprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const Param & param)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint =
    createVehicleFootprint(param.vehicle_info, param.footprint_margin);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory.points) {
    vehicle_footprints.push_back(
      transformVector(local_vehicle_footprint, autoware_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> LaneDepartureChecker::createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  // Create hull from two adjacent vehicle footprints
  std::vector<LinearRing2d> areas;
  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(createHullFromFootprints(footprint1, footprint2));
  }

  return areas;
}

LinearRing2d LaneDepartureChecker::createHullFromFootprints(
  const LinearRing2d & area1, const LinearRing2d & area2)
{
  MultiPoint2d combined;
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

bool LaneDepartureChecker::willLeaveLane(
  const lanelet::ConstLanelets & candidate_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  for (const auto & vehicle_footprint : vehicle_footprints) {
    if (isOutOfLane(candidate_lanelets, vehicle_footprint)) {
      return true;
    }
  }

  return false;
}

bool LaneDepartureChecker::isOutOfLane(
  const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (!isInAnyLane(candidate_lanelets, point)) {
      return true;
    }
  }

  return false;
}
}  // namespace lane_departure_checker
