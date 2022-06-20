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

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/trajectory/path_with_lane_id.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>
#include <vector>

namespace
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
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

PathWithLaneId generateTestTrajectory(
  const size_t num_points, const double point_interval, const double vel = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  PathWithLaneId path_with_lane_id;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    PathPointWithLaneId p;
    p.point.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.point.longitudinal_velocity_mps = vel;
    path_with_lane_id.points.push_back(p);
  }

  return path_with_lane_id;
}
}  // namespace

TEST(trajectory, calcLongitudinalOffsetPoseFromIndex_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;

  const auto path_with_lane_id = generateTestTrajectory(10, 1.0);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPose(PathWithLaneId{}.points, {}, {}), std::invalid_argument);

  // Out of range
  EXPECT_THROW(
    calcLongitudinalOffsetPose(path_with_lane_id.points, path_with_lane_id.points.size() + 1, 1.0),
    std::out_of_range);
  EXPECT_THROW(calcLongitudinalOffsetPose(path_with_lane_id.points, -1, 1.0), std::out_of_range);

  // Same Point
  {
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, 3, 0.0);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 3.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Whole length
  {
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, 0, 9.0);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 9.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Whole length
  {
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, 9, -9.0);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Forward offset
  {
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, 3, 2.25);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 5.25, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Backward offset
  {
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, 3, -2.25);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 0.75, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPose(
      path_with_lane_id.points, 0, calcArcLength(path_with_lane_id.points) + 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPose(
      path_with_lane_id.points, 9, -calcArcLength(path_with_lane_id.points) - 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found(Trajectory size is 1)
  {
    const auto one_point_path = generateTestTrajectory(1, 1.0);
    const auto p_out = calcLongitudinalOffsetPose(one_point_path.points, 0.0, 0.0);

    EXPECT_EQ(p_out, boost::none);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromPoint_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::createPoint;

  const auto path_with_lane_id = generateTestTrajectory(10, 1.0);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPose(PathWithLaneId{}.points, {}, {}), std::invalid_argument);

  // Same Point
  {
    const auto p_src = createPoint(3.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, p_src, 0.0);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 3.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Whole length
  {
    const auto p_src = createPoint(0.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, p_src, 9.0);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 9.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Whole length
  {
    const auto p_src = createPoint(9.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, p_src, -9.0);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Forward offset(No lateral deviation)
  {
    const auto p_src = createPoint(1.25, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, p_src, 2.25);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 3.5, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Forward offset(Lateral deviation)
  {
    const auto p_src = createPoint(-1.25, 1.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, p_src, 4.25);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 3.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Backward offset
  {
    const auto p_src = createPoint(6.25, 1.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, p_src, -2.25);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 4.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // Backward offset(Lateral deviation)
  {
    const auto p_src = createPoint(6.25, -1.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(path_with_lane_id.points, p_src, -4.25);

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 2.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);
  }

  // No found
  {
    const auto p_src = createPoint(0.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(
      path_with_lane_id.points, p_src, calcArcLength(path_with_lane_id.points) + 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found
  {
    const auto p_src = createPoint(9.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(
      path_with_lane_id.points, p_src, -calcArcLength(path_with_lane_id.points) - 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // Out of range(Trajectory size is 1)
  {
    const auto one_point_path = generateTestTrajectory(1, 1.0);
    EXPECT_THROW(
      calcLongitudinalOffsetPose(one_point_path.points, geometry_msgs::msg::Point{}, {}),
      std::out_of_range);
  }
}

TEST(trajectory, insertTargetPoint_PathWithLaneId)
{
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::getPoint;
  using tier4_autoware_utils::insertTargetPoint;

  const auto path_with_lane_id = generateTestTrajectory(10, 1.0);

  // Insert between trajectory front and back
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(3.5, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(path_with_lane_id.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, path_out.points);

    EXPECT_EQ(insert_idx, 4U);
    EXPECT_EQ(path_out.points.size(), path_with_lane_id.points.size() + 1);

    for (size_t i = 0; i < path_out.points.size() - 1; ++i) {
      const auto & p_front = getPoint(path_out.points.at(i));
      const auto & p_back = getPoint(path_out.points.at(i + 1));
      EXPECT_TRUE(calcDistance2d(p_front, p_back) > 1e-3);
    }

    const auto & p_insert = getPoint(path_out.points.at(insert_idx));
    EXPECT_EQ(p_insert.x, p_target.x);
    EXPECT_EQ(p_insert.y, p_target.y);
    EXPECT_EQ(p_insert.z, p_target.z);
  }

  // Overlap base_idx point
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(3.0, 0.0, 0.0);
    const auto insert_idx = insertTargetPoint(3, p_target, path_out.points);

    EXPECT_EQ(insert_idx, 3U);
    EXPECT_EQ(path_out.points.size(), path_with_lane_id.points.size());

    for (size_t i = 0; i < path_out.points.size() - 1; ++i) {
      const auto & p_front = getPoint(path_out.points.at(i));
      const auto & p_back = getPoint(path_out.points.at(i + 1));
      EXPECT_TRUE(calcDistance2d(p_front, p_back) > 1e-3);
    }
  }

  // Overlap base_idx + 1 point
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(4.0, 0.0, 0.0);
    const auto insert_idx = insertTargetPoint(3, p_target, path_out.points);

    EXPECT_EQ(insert_idx, 4U);
    EXPECT_EQ(path_out.points.size(), path_with_lane_id.points.size());

    for (size_t i = 0; i < path_out.points.size() - 1; ++i) {
      const auto & p_front = getPoint(path_out.points.at(i));
      const auto & p_back = getPoint(path_out.points.at(i + 1));
      EXPECT_TRUE(calcDistance2d(p_front, p_back) > 1e-3);
    }
  }

  // Invalid target point(In front of begin point)
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(-1.0, 0.0, 0.0);
    EXPECT_THROW(insertTargetPoint(0, p_target, path_out.points), std::invalid_argument);
  }

  // Invalid target point(Behind of end point)
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(path_with_lane_id.points, p_target);
    EXPECT_THROW(insertTargetPoint(base_idx, p_target, path_out.points), std::invalid_argument);
  }

  // Invalid target point(Huge lateral offset)
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(4.0, 10.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(path_with_lane_id.points, p_target);
    EXPECT_THROW(insertTargetPoint(base_idx, p_target, path_out.points), std::invalid_argument);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory(0, 1.0);
    EXPECT_THROW(
      insertTargetPoint({}, geometry_msgs::msg::Point{}, empty_traj.points), std::invalid_argument);
  }
}

TEST(trajectory, insertTargetPoint_OverlapThreshold_PathWithLaneId)
{
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::getPoint;
  using tier4_autoware_utils::insertTargetPoint;

  constexpr double overlap_threshold = 1e-4;
  const auto path_with_lane_id = generateTestTrajectory(10, 1.0);

  // Insert between trajectory front and back
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(3.0001, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(path_with_lane_id.points, p_target);
    const auto insert_idx =
      insertTargetPoint(base_idx, p_target, path_out.points, overlap_threshold);

    EXPECT_EQ(insert_idx, 4U);
    EXPECT_EQ(path_out.points.size(), path_with_lane_id.points.size() + 1);

    for (size_t i = 0; i < path_out.points.size() - 1; ++i) {
      const auto & p_front = getPoint(path_out.points.at(i));
      const auto & p_back = getPoint(path_out.points.at(i + 1));
      EXPECT_TRUE(calcDistance2d(p_front, p_back) > overlap_threshold);
    }

    const auto & p_insert = getPoint(path_out.points.at(insert_idx));
    EXPECT_EQ(p_insert.x, p_target.x);
    EXPECT_EQ(p_insert.y, p_target.y);
    EXPECT_EQ(p_insert.z, p_target.z);
  }

  // Overlap base_idx point
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(3.00001, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(path_with_lane_id.points, p_target);
    const auto insert_idx =
      insertTargetPoint(base_idx, p_target, path_out.points, overlap_threshold);

    EXPECT_EQ(insert_idx, 3U);
    EXPECT_EQ(path_out.points.size(), path_with_lane_id.points.size());

    for (size_t i = 0; i < path_out.points.size() - 1; ++i) {
      const auto & p_front = getPoint(path_out.points.at(i));
      const auto & p_back = getPoint(path_out.points.at(i + 1));
      EXPECT_TRUE(calcDistance2d(p_front, p_back) > overlap_threshold);
    }
  }

  // Overlap base_idx + 1 point
  {
    auto path_out = path_with_lane_id;

    const auto p_target = createPoint(3.99999, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(path_with_lane_id.points, p_target);
    const auto insert_idx =
      insertTargetPoint(base_idx, p_target, path_out.points, overlap_threshold);

    EXPECT_EQ(insert_idx, 4U);
    EXPECT_EQ(path_out.points.size(), path_with_lane_id.points.size());

    for (size_t i = 0; i < path_out.points.size() - 1; ++i) {
      const auto & p_front = getPoint(path_out.points.at(i));
      const auto & p_back = getPoint(path_out.points.at(i + 1));
      EXPECT_TRUE(calcDistance2d(p_front, p_back) > overlap_threshold);
    }
  }
}
