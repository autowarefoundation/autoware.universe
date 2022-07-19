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

#include "motion_utils/resample/path.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/math/constants.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>
#include <gtest/internal/gtest-port.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>

namespace
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::transformPoint;

constexpr double epsilon = 1e-6;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

PathPoint generateTestPathPoint(
  const double x, const double y, const double z, const double theta = 0.0,
  const double vel_lon = 0.0, const double vel_lat = 0.0, const double heading_rate = 0.0)
{
  PathPoint p;
  p.pose = createPose(x, y, z, 0.0, 0.0, theta);
  p.longitudinal_velocity_mps = vel_lon;
  p.lateral_velocity_mps = vel_lat;
  p.heading_rate_rps = heading_rate;
  return p;
}

template <class T>
T generateTestPath(
  const size_t num_points, const double point_interval, const double vel_lon = 0.0,
  const double vel_lat = 0.0, const double heading_rate_rps = 0.0, const double init_theta = 0.0,
  const double delta_theta = 0.0)
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = vel_lon;
    p.lateral_velocity_mps = vel_lat;
    p.heading_rate_rps = heading_rate_rps;
    traj.points.push_back(p);
  }

  return traj;
}

std::vector<double> generateArclength(const size_t num_points, const double interval)
{
  std::vector<double> resampled_arclength(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    resampled_arclength.at(i) = i * interval;
  }

  return resampled_arclength;
}
}  // namespace

TEST(resample_path, resample_path_by_vector)
{
  using motion_utils::resamplePath;
  // Output is same as input
  {
    auto path = generateTestPath<Path>(10, 1.0, 3.0, 1.0, 0.01);
    std::vector<double> resampled_arclength = generateArclength(10, 1.0);

    {
      const auto resampled_path = resamplePath(path, resampled_arclength);
      EXPECT_NE(resampled_path, boost::none);
      for (size_t i = 0; i < resampled_path->points.size(); ++i) {
        const auto p = resampled_path->points.at(i);
        const auto ans_p = path.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
      }
    }

    path.points.back() =
      generateTestPathPoint(9.0, 0.0, 0.0, tier4_autoware_utils::pi / 3.0, 3.0, 1.0, 0.01);
    {
      const auto resampled_path = resamplePath(path, resampled_arclength);
      EXPECT_NE(resampled_path, boost::none);
      for (size_t i = 0; i < resampled_path->points.size() - 1; ++i) {
        const auto p = resampled_path->points.at(i);
        const auto ans_p = path.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
      }

      const auto p = resampled_path->points.back();
      const auto ans_p = path.points.back();
      const auto ans_quat = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
      EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
      EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
      EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, ans_quat.w, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
    }
  }
}
