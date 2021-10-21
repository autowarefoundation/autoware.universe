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

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"

#include "utils.hpp"

TEST(buildPathLanelet, nominal)
{
  using behavior_velocity_planner::occlusion_spot_utils::buildPathLanelet;
  lanelet::ConstLanelet path_lanelet;
  int first_index;
  /* straight diagonal path
      0 1 2 3 4
    0 x
    1   x
    2     x
    3       x
    4         x
  */
  autoware_planning_msgs::msg::PathWithLaneId path = test::generatePath(0, 0, 4, 4, 5);
  first_index = 0;
  path_lanelet = buildPathLanelet(path, first_index, 1000);
  ASSERT_EQ(path_lanelet.centerline2d().front().x(), 0.0);
  ASSERT_EQ(path_lanelet.centerline2d().front().y(), 0.0);
  ASSERT_NE(path_lanelet.centerline2d().back().x(), 4.0);
  ASSERT_NE(path_lanelet.centerline2d().back().y(), 4.0);
  // changing the first path index
  first_index = 1;
  path_lanelet = buildPathLanelet(path, first_index, 1000);
  ASSERT_EQ(path_lanelet.centerline2d().front().x(), 1.0);
  ASSERT_EQ(path_lanelet.centerline2d().front().y(), 1.0);
  ASSERT_NE(path_lanelet.centerline2d().back().x(), 4.0);
  ASSERT_NE(path_lanelet.centerline2d().back().y(), 4.0);
  first_index = 4;
  path_lanelet = buildPathLanelet(path, first_index, 1000);
  // ASSERT_EQ(path_lanelet.centerline2d().front(), path_lanelet.centerline2d().back());
}

TEST(calcVelocityAndHeightToPossibleCollision, TooManyPossibleCollisions)
{
  using behavior_velocity_planner::occlusion_spot_utils::calcVelocityAndHeightToPossibleCollision;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  std::vector<PossibleCollisionInfo> possible_collisions;
  // make a path with 2000 points from x=0 to x=4
  autoware_planning_msgs::msg::PathWithLaneId path = test::generatePath(0.0, 3.0, 4.0, 3.0, 2000);
  // make 2000 possible collision from x=0 to x=10
  test::generatePossibleCollisions(possible_collisions, 0.0, 3.0, 4.0, 3.0, 2000);

  /**
 * @brief too many possible collisions on path
 *
 * Ego -col-col-col-col-col-col-col--col-col-col-col-col-col-col-col-col-> path
 *
 */

  auto start_naive = high_resolution_clock::now();
  calcVelocityAndHeightToPossibleCollision(path, possible_collisions);

  auto end_naive = high_resolution_clock::now();
  // 2000 path * 2000 possible collisions
  EXPECT_EQ(possible_collisions.size(), size_t{2000});
  EXPECT_EQ(path.points.size(), size_t{2000});
  EXPECT_TRUE(duration_cast<microseconds>(end_naive - start_naive).count() < 500);
  std::cout << " runtime (microsec) " <<
    duration_cast<microseconds>(end_naive - start_naive).count() << std::endl;
}

TEST(createPossibleCollisionBehindParkedVehicle, TooManyPathPointsAndObstacles)
{
  using behavior_velocity_planner::occlusion_spot_utils::createPossibleCollisionBehindParkedVehicle;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  // make a path with 200 points from x=0 to x=200
  autoware_planning_msgs::msg::PathWithLaneId path = test::generatePath(0.0, 3.0, 200.0, 3.0, 100);
  int first_path_index(0);
  // There is a parked bus,car,truck along with ego path.
  // Ignore vehicle dimensions to simplify test
  std::cout << " 6 -   |TRU|   |   |     -> truck is ignored because of lateral distance   \n" <<
    " 5 -   |   |   |   |                    \n" <<
    " 4 -   |CAR|   |   |     -> considered  \n" <<
    " 3Ego--|---|--path-->   (2000 points)   \n" <<
    " ＝＝median strip====                   \n" <<
    " 2 -   |   |   |SUB|     -> bus is ignored because of opposite direction \n" <<
    " 1 -   |   |   |   |                    \n" <<
    " 0 | 1 | 2 | 3 | 4 | \n";

  autoware_perception_msgs::msg::DynamicObjectArray obj_arr;
  autoware_perception_msgs::msg::DynamicObject obj;
  obj.shape.dimensions.x = 0.0;
  obj.shape.dimensions.y = 0.0;
  tf2::Quaternion q;
  obj.state.pose_covariance.pose.orientation = tf2::toMsg(q);
  obj.state.orientation_reliable = true;
  obj.state.twist_reliable = true;
  obj.state.twist_covariance.twist.linear.x = 0;

  // car
  obj.state.pose_covariance.pose.position.x = 2.5;
  obj.state.pose_covariance.pose.position.y = 4.0;
  obj.semantic.type = autoware_perception_msgs::msg::Semantic::CAR;
  const size_t num_car = 30;
  for (size_t i = 0; i < num_car; i++) {
    obj_arr.objects.emplace_back(obj);
  }

  // truck
  obj.state.pose_covariance.pose.position.x = 2.5;
  obj.state.pose_covariance.pose.position.y = 6.0;
  obj.semantic.type = autoware_perception_msgs::msg::Semantic::TRUCK;
  obj_arr.objects.emplace_back(obj);

  // bus
  q.setRPY(0, 0, M_PI);
  obj.state.pose_covariance.pose.position.x = 4.5;
  obj.state.pose_covariance.pose.position.y = 2.0;
  obj.state.pose_covariance.pose.orientation = tf2::toMsg(q);
  obj.semantic.type = autoware_perception_msgs::msg::Semantic::BUS;
  obj_arr.objects.emplace_back(obj);

  // Set parameters: enable sidewalk obstacles
  behavior_velocity_planner::occlusion_spot_utils::PlannerParam parameters;
  parameters.vehicle_info.baselink_to_front = 0.0;
  parameters.vehicle_info.vehicle_width = 0.0;
  parameters.detection_area_length = 100;
  parameters.pedestrian_vel = 1.6;
  parameters.lateral_distance_thr = 2.5;

  auto obj_arr_ptr =
    std::make_shared<autoware_perception_msgs::msg::DynamicObjectArray>(obj_arr);
  auto start_naive = high_resolution_clock::now();
  std::vector<behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo>
  possible_collisions;
  createPossibleCollisionBehindParkedVehicle(
    possible_collisions, path, parameters, first_path_index, obj_arr_ptr);
  auto end_naive = high_resolution_clock::now();
  // the possible collision is inserted and
  // it's ego distance and obstacle distance is the same as (x,|y|)
  ASSERT_EQ(possible_collisions.size(), static_cast<size_t>(num_car));
  EXPECT_EQ(possible_collisions[0].arc_lane_dist_at_collision.length, 2.5);
  EXPECT_EQ(possible_collisions[0].arc_lane_dist_at_collision.distance, 1);
  std::cout << " runtime (microsec) " <<
    duration_cast<microseconds>(end_naive - start_naive).count() << std::endl;
}
