// Copyright 2024 TIER IV, Inc.
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

#include "dynamic_obstacle.hpp"
#include "path_utils.hpp"
#include "scene.hpp"
#include "utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <tier4_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using autoware_perception_msgs::msg::ObjectClassification;
using geometry_msgs::msg::Point;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

using autoware::behavior_velocity_planner::applyVoxelGridFilter;
using autoware::behavior_velocity_planner::createPredictedPath;
using autoware::behavior_velocity_planner::createQuaternionFacingToTrajectory;
using autoware::behavior_velocity_planner::run_out_utils::createExtendPathPoint;
class TestDynamicObstacle : public ::testing::Test
{
  void SetUp() override {}
};
namespace autoware::behavior_velocity_planner
{
TEST_F(TestDynamicObstacle, testCreateQuaternionFacingToTrajectory)
{
  constexpr size_t n_path_points{10};
  PathPointsWithLaneId path;
  PathPointWithLaneId base_point;
  for (size_t i = 0; i < n_path_points; ++i) {
    const PathPointWithLaneId p = createExtendPathPoint(static_cast<double>(i), base_point);
    path.push_back(p);
  }

  /*
                 path
                  |           ^ x
                  |           |
                  |    y      |
                  |    <------- ref frame
                  |
                  |
     (2.0,0.0)    |<------0 (2.0,-2.0) point
                  |    ^
                  |    |
                  | 90 deg yaw
  */

  {
    geometry_msgs::msg::Point point;
    point.x = 2.0;
    point.y = -2.0;

    const auto quaternion_facing_traj = createQuaternionFacingToTrajectory(path, point);
    const auto rpy = autoware::universe_utils::getRPY(quaternion_facing_traj);
    const auto yaw = autoware::universe_utils::normalizeRadian(rpy.z);
    EXPECT_DOUBLE_EQ(yaw, M_PI_2);
  }

  {
    geometry_msgs::msg::Point point;
    point.x = 2.75;  // path resolution is 1.0, this makes sure the point is "behind" the closest
                     // point on the path.
    point.y = -0.25;

    const auto quaternion_facing_traj = createQuaternionFacingToTrajectory(path, point);
    const auto rpy = autoware::universe_utils::getRPY(quaternion_facing_traj);
    const auto yaw = autoware::universe_utils::normalizeRadian(rpy.z);
    EXPECT_DOUBLE_EQ(std::abs(yaw), M_PI_2);
  }
}

TEST_F(TestDynamicObstacle, testCreatePredictedPath)
{
  using autoware::behavior_velocity_planner::run_out_utils::isSamePoint;

  constexpr float time_step{0.1};
  constexpr float max_velocity_mps{3.0};
  constexpr float max_prediction_time{5.0};
  geometry_msgs::msg::Pose initial_pose;

  const auto traj =
    createPredictedPath(initial_pose, time_step, max_velocity_mps, max_prediction_time);
  EXPECT_EQ(traj.size(), static_cast<size_t>(max_prediction_time / time_step));
  auto last_pose = traj.back();
  geometry_msgs::msg::Point expected_point;
  expected_point.x = max_prediction_time * max_velocity_mps - time_step * max_velocity_mps;
  EXPECT_TRUE(std::abs(expected_point.x - last_pose.position.x) < 1e-3);
}

TEST_F(TestDynamicObstacle, testApplyVoxelGridFilter)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  constexpr int number_points_in_axis{10};
  for (size_t i = 0; i < number_points_in_axis; ++i) {
    for (size_t j = 0; j < number_points_in_axis; ++j) {
      for (size_t k = 0; k < number_points_in_axis; ++k) {
        pcl::PointXYZ p;
        p.x = i * 0.025;
        p.y = j * 0.025;
        p.z = k * 0.025;
        point_cloud.push_back(p);
      }
    }
  }

  {
    auto filtered_point_cloud = applyVoxelGridFilter(point_cloud);
    EXPECT_FALSE(filtered_point_cloud.empty());
    EXPECT_TRUE(filtered_point_cloud.size() < point_cloud.size());
    const bool points_have_no_height = std::all_of(
      filtered_point_cloud.begin(), filtered_point_cloud.end(),
      [](const auto & p) { return std::abs(p.z) < std::numeric_limits<double>::epsilon(); });
    EXPECT_TRUE(points_have_no_height);
  }

  sensor_msgs::msg::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(point_cloud, ros_pointcloud);

  {
    auto filtered_point_cloud = applyVoxelGridFilter(ros_pointcloud);
    EXPECT_FALSE(filtered_point_cloud.empty());
    EXPECT_TRUE(filtered_point_cloud.size() < point_cloud.size());
    const bool points_have_no_height = std::all_of(
      filtered_point_cloud.begin(), filtered_point_cloud.end(),
      [](const auto & p) { return std::abs(p.z) < std::numeric_limits<double>::epsilon(); });
    EXPECT_TRUE(points_have_no_height);
  }
}

}  // namespace autoware::behavior_velocity_planner
