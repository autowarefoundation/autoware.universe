// Copyright 2020-2021 Embotech AG, Zurich, Switzerland, Arm Limited
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

#include <gtest/gtest.h>
#include <recordreplay_planner_nodes/recordreplay_planner_node.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <motion_testing/motion_testing.hpp>
#include <rclcpp/rclcpp.hpp>
#include <motion_common/config.hpp>

#include <chrono>
#include <algorithm>
#include <memory>

#include "object_collision_estimator_nodes/object_collision_estimator_node.hpp"


using motion::planning::recordreplay_planner_nodes::RecordReplayPlannerNode;
using motion::planning::object_collision_estimator_nodes::ObjectCollisionEstimatorNode;
using motion::motion_testing::make_state;
using std::chrono::system_clock;
using autoware_auto_planning_msgs::msg::Trajectory;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;

using motion::motion_common::VehicleConfig;

TEST(MytestBase, Basic)
{
  const auto heading_weight = 0.1;
  const auto min_record_distance = 0.0;
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions node_options_rr, node_options_ob;
  node_options_ob.append_parameter_override("vehicle.cg_to_front_m", 1.0);
  node_options_ob.append_parameter_override("vehicle.cg_to_rear_m", 1.0);
  node_options_ob.append_parameter_override("vehicle.front_corner_stiffness", 0.5);
  node_options_ob.append_parameter_override("vehicle.rear_corner_stiffness", 0.5);
  node_options_ob.append_parameter_override("vehicle.mass_kg", 1500.0);
  node_options_ob.append_parameter_override("vehicle.yaw_inertia_kgm2", 12.0);
  node_options_ob.append_parameter_override("vehicle.width_m", 2.0);
  node_options_ob.append_parameter_override("vehicle.front_overhang_m", 0.5);
  node_options_ob.append_parameter_override("vehicle.rear_overhang_m", 0.2);
  node_options_ob.append_parameter_override("safety_factor", 2.0);
  node_options_ob.append_parameter_override("stop_margin", 5.0);
  node_options_ob.append_parameter_override("min_obstacle_dimension_m", 0.0004);
  node_options_ob.append_parameter_override("trajectory_smoother.kernel_std", 5.0);
  node_options_ob.append_parameter_override("trajectory_smoother.kernel_size", 25);
  node_options_ob.append_parameter_override("staleness_threshold_ms", 500);
  node_options_ob.append_parameter_override("target_frame_id", "map");
  auto object_collision_estimator_node = std::make_shared<ObjectCollisionEstimatorNode>(
    node_options_ob);

  node_options_rr.append_parameter_override("heading_weight", heading_weight);
  node_options_rr.append_parameter_override("min_record_distance", min_record_distance);
  node_options_rr.append_parameter_override("enable_object_collision_estimator", true);
  node_options_rr.append_parameter_override("enable_object_collision_estimator", true);
  node_options_rr.append_parameter_override("goal_distance_threshold_m", 0.75);
  node_options_rr.append_parameter_override("loop_trajectory", false);
  node_options_rr.append_parameter_override("loop_max_gap_m", 0.0);
  node_options_rr.append_parameter_override(
    "goal_angle_threshold_rad",
    autoware::common::types::PI_2);
  node_options_rr.append_parameter_override("skip_first_velocity", false);
  auto plannernode = std::make_shared<RecordReplayPlannerNode>(node_options_rr);

  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  const auto publisher = std::make_shared<rclcpp::Node>(
    "recordreplay_node_testpublisher");
  const auto pub = publisher->create_publisher<State>(
    "vehicle_state", rclcpp::QoS{10}.transient_local(), PubAllocT{});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(plannernode);
  exec.add_node(publisher);

  auto dummy_state = std::make_shared<State>();
  pub->publish(*dummy_state);
  EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));

  // TODO(s.me) actually do what I planned on doing in the launch_testing file here.
  // This is tracked by https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/issues/273.

  rclcpp::shutdown();
}
