// Copyright 2022 The Autoware Foundation
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "collision_free_path_planner/node.hpp"
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
// #include "collision_free_path_planner/mpt_optimizer.hpp"

namespace collision_free_path_planner
{

namespace
{
ReferencePoint createRefPoint(
  const double x, const double y, const double z, const double yaw, const double v)
{
  ReferencePoint ref_point;
  ref_point.p.x = x;
  ref_point.p.y = y;
  ref_point.p.z = z;
  ref_point.yaw = yaw;
  ref_point.v = v;

  return ref_point;
}
}  // namespace

TEST(CollisionFreePathPlanner, MPTOptimizer)
{
  rclcpp::init(0, nullptr);

  // Pass default parameter file to the node
  const auto share_dir =
    ament_index_cpp::get_package_share_directory("collision_free_path_planner");
  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args",                                                                  //
     "--params-file", share_dir + "/param/collision_free_path_planner.param.yaml",  //
     "--params-file", share_dir + "/param/test_vehicle_info.param.yaml",            //
     "--params-file", share_dir + "/param/test_nearest_search.param.yaml"});

  // create node
  // std::shared_ptr<CollisionFreePathPlanner> node =
  // std::make_shared<CollisionFreePathPlanner>(node_options);
  auto node = CollisionFreePathPlanner(node_options);
  const auto & mpt = *node.mpt_optimizer_ptr_;
  DebugData debug_data;

  ReferencePoint ref_point1 = createRefPoint(1, 0, 0, 1, 1);
  ReferencePoint ref_point2 = createRefPoint(2, 0, 0, 2, 2);
  ReferencePoint ref_point3 = createRefPoint(3, 0, 0, 3, 3);

  {  // test various points arguments
    std::vector<ReferencePoint> empty_ref_points{};
    std::vector<ReferencePoint> single_ref_points{ReferencePoint{}};
    std::vector<ReferencePoint> two_identical_ref_points{ReferencePoint{}, ReferencePoint{}};
    std::vector<ReferencePoint> two_different_ref_points{ref_point1, ref_point2};
    std::vector<ReferencePoint> three_identical_ref_points{ReferencePoint{}, ReferencePoint{}};
    std::vector<ReferencePoint> three_different_ref_points{ref_point1, ref_point2, ref_point3};

    EXPECT_NO_THROW(mpt.generateMPTMatrix(empty_ref_points, debug_data));
    EXPECT_NO_THROW(mpt.generateMPTMatrix(single_ref_points, debug_data));
    EXPECT_NO_THROW(mpt.generateMPTMatrix(two_identical_ref_points, debug_data));
    EXPECT_NO_THROW(mpt.generateMPTMatrix(two_different_ref_points, debug_data));
    EXPECT_NO_THROW(mpt.generateMPTMatrix(three_identical_ref_points, debug_data));
    EXPECT_NO_THROW(mpt.generateMPTMatrix(three_different_ref_points, debug_data));

    /*
    EXPECT_NO_THROW(mpt.calcOrientation(empty_ref_points));
    EXPECT_NO_THROW(mpt.calcOrientation(single_ref_points));
    EXPECT_NO_THROW(mpt.calcOrientation(two_identical_ref_points));
    */
  }

  {  // test nullptr arguments
  }

  // MPTOptimizer(node, enable_debug_info_, ego_nearest_param_, vehicle_info, traj_param_,
  // vehicle_param_);
  EXPECT_EQ(0, 0);
}
}  // namespace collision_free_path_planner
