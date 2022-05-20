// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "freespace_planning_algorithms/rrtstar_core.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <stack>

bool checkAllNodeConnected(const rrtstar_core::RRTStar & tree)
{
  const auto & nodes = tree.getNodes();
  rrtstar_core::NodeConstSharedPtr root = nodes.front();

  std::stack<rrtstar_core::NodeConstSharedPtr> node_stack;
  node_stack.push(root);

  size_t visit_count = 0;
  while (!node_stack.empty()) {
    const auto & node_here = node_stack.top();
    node_stack.pop();
    visit_count += 1;
    for (const auto & child : node_here->childs) {
      node_stack.push(child);
    }
  }
  return nodes.size() == visit_count;
}

TEST(RRTStarCore, WithInformedOption)
{
  const rrtstar_core::Pose x_start{0.1, 0.1, 0};
  const rrtstar_core::Pose x_goal{0.5, 0.9, 0.};

  const rrtstar_core::Pose x_lo{0, 0, -6.28};
  const rrtstar_core::Pose x_hi{1., 1., +6.28};

  auto is_collision_free = [](const rrtstar_core::Pose & p) {
    const double radius_squared = (p.x - 0.5) * (p.x - 0.5) + (p.y - 0.5) * (p.y - 0.5);
    return radius_squared > 0.09;
  };
  const auto resolution = 0.01;
  const auto cspace = rrtstar_core::CSpace(x_lo, x_hi, 0.1, is_collision_free);
  auto algo = rrtstar_core::RRTStar(x_start, x_goal, 0.2, resolution, true, cspace);

  clock_t start = clock();
  for (int i = 0; i < 2000; i++) {
    algo.extend();
  }
  clock_t end = clock();
  std::cout << "elapsed time : " << (end - start) / 1000.0 << " [msec]" << std::endl;
  algo.dumpState("/tmp/rrt_result.txt");
  EXPECT_TRUE(checkAllNodeConnected(algo));

  {  // testing
    const auto is_pose_equal =
      [](const rrtstar_core::Pose & p0, const rrtstar_core::Pose & p1) -> bool {
      const double eps = 1e-8;
      if (abs(p0.x - p1.x) > eps) {
        return false;
      }
      if (abs(p0.y - p1.y) > eps) {
        return false;
      }
      if (abs(p0.yaw - p1.yaw) > eps) {
        return false;
      }
    };
    const auto & nodes = algo.getNodes();

    // check all path (including result path) feasibility
    bool is_all_path_feasible = true;
    for (const auto & node : nodes) {
      if (!node.parent_idx) {
        continue;
      }
      const auto & node_parent = nodes[*node.parent_idx];
      std::vector<rrtstar_core::Pose> mid_poses;
      cspace.sampleWayPoints_child2parent(node.pose, node_parent.pose, resolution, mid_poses);

      // check feasibility
      for (const auto & pose : mid_poses) {
        if (!cspace.isValidPose(pose)) {
          is_all_path_feasible = false;
        }
      }
    }
    EXPECT_TRUE(is_all_path_feasible);
  }

  {  // check solution trajectory
    const auto waypoints = algo.sampleSolutionWaypoints(resolution);
    EXPECT_GT(waypoints.size(), 2);

    bool is_feasible = true;
    for (const auto & p : waypoints) {
      if (!cspace.isValidPose(p)) {
        is_feasible = false;
      }
    }
    EXPECT_TRUE(is_feasible);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
