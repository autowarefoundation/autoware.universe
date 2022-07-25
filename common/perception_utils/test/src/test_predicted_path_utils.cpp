// Copyright 2020 TIER IV, Inc.
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

#include "perception_utils/predicted_path_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>

using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Point3d;

constexpr double epsilon = 1e-06;

namespace
{
using autoware_auto_perception_msgs::msg::PredictedPath;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::transformPoint;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

PredictedPath createTestPredictedPath(
  const size_t num_points, const double point_time_interval, const double vel,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(point_time_interval);

  const double point_interval = vel * point_time_interval;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    const auto p = createPose(x, y, 0.0, 0.0, 0.0, theta);
    path.path.push_back(p);
  }
  return path;
}
}  // namespace

TEST(predicted_path_utils, testCalcInterpolatedPose)
{
  using perception_utils::calcInterpolatedPose;
  using tier4_autoware_utils::createQuaternionFromRPY;
  using tier4_autoware_utils::createQuaternionFromYaw;
  using tier4_autoware_utils::deg2rad;

  const auto path = createTestPredictedPath(100, 0.1, 1.0);

  {
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
    for (double t = 0.0; t < 10.0; t += 1.0) {
      const auto p = calcInterpolatedPose(path, t);

      EXPECT_NE(p, boost::none);
      EXPECT_NEAR(p->position.x, t * 1.0, epsilon);
      EXPECT_NEAR(p->position.y, 0.0, epsilon);
      EXPECT_NEAR(p->position.z, 0.0, epsilon);
      EXPECT_NEAR(p->orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p->orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p->orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p->orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Interpolation
  {
    // Negative time
    {
      const auto p = calcInterpolatedPose(path, -1.0);
      EXPECT_EQ(p, boost::none);
    }

    // Over the time horizon
    {
      const auto p = calcInterpolatedPose(path, 10.0 + 1e-6);
      EXPECT_EQ(p, boost::none);
    }

    // Empty Path
    {
      PredictedPath empty_path;
      const auto p = calcInterpolatedPose(empty_path, -1.0);
      EXPECT_EQ(p, boost::none);
    }
  }
}
