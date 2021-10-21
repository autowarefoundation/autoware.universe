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

#include <limits>

#include "gtest/gtest.h"

#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"
#include "scene_module/occlusion_spot/risk_predictive_braking.hpp"

#include "utils.hpp"

TEST(calculatePredictiveBrakingVelocity, min_max)
{
  using behavior_velocity_planner::occlusion_spot_utils::calculatePredictiveBrakingVelocity;
  const double inf = std::numeric_limits<double>::max();
  std::cout << "PBS(v0,dist,pbs) of (10,2,-inf) --> 0[m/s]  \n";
  // lower bound ego_vel
  ASSERT_EQ(calculatePredictiveBrakingVelocity(0, 2, -inf), 0);
  std::cout << "PBS(v0,dist,pbs) of (10,inf,0) --> 10[m/s]  \n";
  // upper bound
  ASSERT_EQ(calculatePredictiveBrakingVelocity(10, inf, 0), 10);
}

TEST(calculateSafeRPBVelocity, min_max)
{
  using behavior_velocity_planner::occlusion_spot_utils::calculateSafeRPBVelocity;
  // lower bound ttc_vir = 0
  const double t_buff = 0.5;
  double d_obs = 0.5;
  double v_obs = 1.0;
  ASSERT_EQ(calculateSafeRPBVelocity(t_buff, d_obs, v_obs, -5.0), 0.0);
  // lower bound ebs_decel = 0
  ASSERT_EQ(calculateSafeRPBVelocity(1.0, 0.5, 0.5, 0), 0.0);
}

TEST(getPBSLimitedRPBVelocity, min_max)
{
  using behavior_velocity_planner::occlusion_spot_utils::getPBSLimitedRPBVelocity;
  const double inf = std::numeric_limits<double>::max();
  // upper bound rpb_vel
  ASSERT_EQ(getPBSLimitedRPBVelocity(inf, inf, inf, inf), inf);
  // lower bound org_vel = 0
  ASSERT_EQ(getPBSLimitedRPBVelocity(inf, inf, inf, 0), 0.0);
  // lower bound min = 0
  ASSERT_EQ(getPBSLimitedRPBVelocity(inf, inf, 0, inf), inf);
}

TEST(insertSafeVelocityToPath, dont_insert_last_point)
{
  using behavior_velocity_planner::occlusion_spot_utils::insertSafeVelocityToPath;
  const int num_path = 3;
  geometry_msgs::msg::Pose pose{};
  pose.position.x = 1;
  pose.position.y = 0;
  pose.position.z = 0;
  double safe_vel = 10;
  behavior_velocity_planner::occlusion_spot_utils::PlannerParam param;
  param.angle_thr = 1.0;
  param.dist_thr = 10.0;
  autoware_planning_msgs::msg::PathPointWithLaneId p{};
  autoware_planning_msgs::msg::PathWithLaneId path =
    test::generatePath(0.0, 0.0, static_cast<double>(num_path), 0.0, num_path);
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  insertSafeVelocityToPath(pose, safe_vel, param, &path);
  ASSERT_EQ(path.points.size(), static_cast<size_t>(num_path + 1));
  pose.position.x = static_cast<double>(num_path + 1);
  insertSafeVelocityToPath(pose, safe_vel, param, &path);
  ASSERT_EQ(path.points.size(), static_cast<size_t>(num_path + 1));
}
