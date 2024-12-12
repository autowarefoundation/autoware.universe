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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_start_planner_module/geometric_pull_out.hpp>
#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/lane_departure_checker/lane_departure_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::GeometricPullOut;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::lane_departure_checker::LaneDepartureChecker;
using autoware::test_utils::get_absolute_path_to_config;
using autoware_planning_msgs::msg::LaneletRoute;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId;

namespace autoware::behavior_path_planner
{

class TestGeometricPullOut : public ::testing::Test
{
public:
  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose, PlannerDebugData & planner_debug_data)
  {
    return geometric_pull_out->plan(start_pose, goal_pose, planner_debug_data);
  }

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  std::shared_ptr<GeometricPullOut> geometric_pull_out;
  std::shared_ptr<LaneDepartureChecker> lane_departure_checker;

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    init_param();
    init_module();
  }
  void TearDown() override { rclcpp::shutdown(); }

private:
  void init_param()
  {
    auto node_options = get_node_options();
    auto node = rclcpp::Node::make_shared("geometric_pull_out", node_options);

    vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

    // lanelet map
    const auto shoulder_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "road_shoulder/lanelet2_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(shoulder_map_path, 0.5);
    // load map
    route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);

    lane_departure_checker = std::make_shared<LaneDepartureChecker>();
    lane_departure_checker->setVehicleInfo(vehicle_info);

    auto parameters = std::make_shared<StartPlannerParameters>();
    auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();

    autoware::lane_departure_checker::Param lane_departure_checker_params{};
    lane_departure_checker_params.footprint_extra_margin =
      parameters->lane_departure_check_expansion_margin;

    lane_departure_checker->setParam(lane_departure_checker_params);
    parameters->parallel_parking_parameters.pull_out_max_steer_angle = 0.35;
    parameters->parallel_parking_parameters.pull_out_arc_path_interval = 1.0;
    parameters->parallel_parking_parameters.center_line_path_interval = 1.0;
    parameters->th_moving_object_velocity = 1.0;

    geometric_pull_out =
      std::make_shared<GeometricPullOut>(*node, *parameters, lane_departure_checker, time_keeper);
  }

  rclcpp::NodeOptions get_node_options() const
  {
    auto node_options = rclcpp::NodeOptions{};

    const auto common_param =
      get_absolute_path_to_config("autoware_test_utils", "test_common.param.yaml");
    const auto nearest_search_param =
      get_absolute_path_to_config("autoware_test_utils", "test_nearest_search.param.yaml");
    const auto vehicle_info_param =
      get_absolute_path_to_config("autoware_test_utils", "test_vehicle_info.param.yaml");
    const auto behavior_path_planner_param = get_absolute_path_to_config(
      "autoware_behavior_path_planner", "behavior_path_planner.param.yaml");
    const auto drivable_area_expansion_param = get_absolute_path_to_config(
      "autoware_behavior_path_planner", "drivable_area_expansion.param.yaml");
    const auto scene_module_manager_param = get_absolute_path_to_config(
      "autoware_behavior_path_planner", "scene_module_manager.param.yaml");
    const auto start_planner_param = get_absolute_path_to_config(
      "autoware_behavior_path_start_planner_module", "start_planner.param.yaml");

    autoware::test_utils::updateNodeOptions(
      node_options,
      {common_param, nearest_search_param, vehicle_info_param, behavior_path_planner_param,
       drivable_area_expansion_param, scene_module_manager_param, start_planner_param});
    return node_options;
  }

  void init_module() {}
};

TEST_F(TestGeometricPullOut, GenerateValidGeometricPullOutPath)
{
  // Given: A valid start pose and goal pose
  const geometry_msgs::msg::Pose start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.181).y(362.164).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709650).w(
          0.704554));

  const geometry_msgs::msg::Pose goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(365.658).y(507.253).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.705897).w(
          0.708314));

  // And: Required planner configuration is set up
  PlannerData planner_data;
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose = start_pose;
  odometry->header.frame_id = "map";
  planner_data.self_odometry = odometry;

  const auto route = makeBehaviorRouteFromLaneId(
    4619, 4635, "autoware_test_utils", "road_shoulder/lanelet2_map.osm");
  route_handler->setRoute(route);
  planner_data.route_handler = route_handler;

  planner_data.parameters.backward_path_length = 5.0;
  planner_data.parameters.forward_path_length = 100.0;
  planner_data.parameters.wheel_base = 2.79;
  planner_data.parameters.wheel_tread = 1.64;
  planner_data.parameters.front_overhang = 1.0;
  planner_data.parameters.left_over_hang = 0.128;

  geometric_pull_out->setPlannerData(std::make_shared<PlannerData>(planner_data));

  // When: Pull out path is planned
  PlannerDebugData debug_data;
  auto result = plan(start_pose, goal_pose, debug_data);

  // Then: A valid geometric pull out path should be generated
  ASSERT_TRUE(result.has_value()) << "Geometric pull out path generation failed";
  EXPECT_FALSE(result->partial_paths.empty())
    << "Generated pull out path contains no partial paths";
  EXPECT_EQ(debug_data.conditions_evaluation.back(), "success")
    << "Geometric pull out path planning did not succeed";
}

}  // namespace autoware::behavior_path_planner
