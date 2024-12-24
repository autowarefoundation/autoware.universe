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
#include <autoware/behavior_path_start_planner_module/freespace_pull_out.hpp>
#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/lane_departure_checker/lane_departure_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::FreespacePullOut;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::lane_departure_checker::LaneDepartureChecker;
using autoware::test_utils::get_absolute_path_to_config;
using autoware_planning_msgs::msg::LaneletRoute;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId;

namespace autoware::behavior_path_planner
{

class TestFreespacePullOut : public ::testing::Test
{
public:
  std::optional<PullOutPath> call_plan(
    const Pose & start_pose, const Pose & goal_pose, PlannerDebugData & planner_debug_data)
  {
    return freespace_pull_out_->plan(start_pose, goal_pose, planner_debug_data);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("freespace_pull_out", make_node_options());

    initialize_lane_departure_checker();
    freespace_pull_out_planner();
  }

  void TearDown() override { rclcpp::shutdown(); }

  PlannerData make_planner_data(
    const Pose & start_pose, const int route_start_lane_id, const int route_goal_lane_id)
  {
    PlannerData planner_data;
    planner_data.init_parameters(*node_);

    // Load a sample lanelet map and create a route handler
    const auto shoulder_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "road_shoulder/lanelet2_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(shoulder_map_path, 0.5);
    auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);

    // Set up current odometry at start pose
    auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
    odometry->pose.pose = start_pose;
    odometry->header.frame_id = "map";
    planner_data.self_odometry = odometry;

    // Setup route
    const auto route = makeBehaviorRouteFromLaneId(
      route_start_lane_id, route_goal_lane_id, "autoware_test_utils",
      "road_shoulder/lanelet2_map.osm");
    route_handler->setRoute(route);

    // Update planner data with the route handler
    planner_data.route_handler = route_handler;
    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header.frame_id = "map";
    costmap.info.width = 200;
    costmap.info.height = 200;
    costmap.info.resolution = 0.5;
    costmap.info.origin.position.x = 250.0;
    costmap.info.origin.position.y = 230.0;
    costmap.data = std::vector<int8_t>(costmap.info.width * costmap.info.height, 0);
    planner_data.costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap);

    return planner_data;
  }

  // Member variables
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<FreespacePullOut> freespace_pull_out_;
  std::shared_ptr<LaneDepartureChecker> lane_departure_checker_;

private:
  rclcpp::NodeOptions make_node_options() const
  {
    // Load common configuration files
    auto node_options = rclcpp::NodeOptions{};

    const auto common_param_path =
      get_absolute_path_to_config("autoware_test_utils", "test_common.param.yaml");
    const auto nearest_search_param_path =
      get_absolute_path_to_config("autoware_test_utils", "test_nearest_search.param.yaml");
    const auto vehicle_info_param_path =
      get_absolute_path_to_config("autoware_test_utils", "test_vehicle_info.param.yaml");
    const auto behavior_path_planner_param_path = get_absolute_path_to_config(
      "autoware_behavior_path_planner", "behavior_path_planner.param.yaml");
    const auto drivable_area_expansion_param_path = get_absolute_path_to_config(
      "autoware_behavior_path_planner", "drivable_area_expansion.param.yaml");
    const auto scene_module_manager_param_path = get_absolute_path_to_config(
      "autoware_behavior_path_planner", "scene_module_manager.param.yaml");
    const auto start_planner_param_path = get_absolute_path_to_config(
      "autoware_behavior_path_start_planner_module", "start_planner.param.yaml");

    autoware::test_utils::updateNodeOptions(
      node_options, {common_param_path, nearest_search_param_path, vehicle_info_param_path,
                     behavior_path_planner_param_path, drivable_area_expansion_param_path,
                     scene_module_manager_param_path, start_planner_param_path});

    return node_options;
  }

  void initialize_lane_departure_checker()
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();
    lane_departure_checker_ = std::make_shared<LaneDepartureChecker>();
    lane_departure_checker_->setVehicleInfo(vehicle_info);

    autoware::lane_departure_checker::Param lane_departure_checker_params{};
    lane_departure_checker_->setParam(lane_departure_checker_params);
  }
  void freespace_pull_out_planner()
  {
    auto parameters = StartPlannerParameters::init(*node_);
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();
    auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();
    freespace_pull_out_ =
      std::make_shared<FreespacePullOut>(*node_, parameters, vehicle_info, time_keeper);
  }
};

TEST_F(TestFreespacePullOut, GenerateValidFreespacePullOutPath)
{
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(299.796).y(303.529).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(-0.748629).w(
          0.662990));

  const auto goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(270.789).y(246.749).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(-0.727585).w(
          0.686018));

  const auto planner_data = make_planner_data(start_pose, 508, 720);

  freespace_pull_out_->setPlannerData(std::make_shared<PlannerData>(planner_data));

  // Plan the pull out path
  PlannerDebugData debug_data;
  auto result = call_plan(start_pose, goal_pose, debug_data);

  // Assert that a valid shift pull out path is generated
  ASSERT_TRUE(result.has_value()) << "shift pull out path generation failed.";
  EXPECT_EQ(result->partial_paths.size(), 2UL)
    << "Generated shift pull out path does not have the expected number of partial paths.";
  EXPECT_EQ(debug_data.conditions_evaluation.back(), "success")
    << "shift pull out path planning did not succeed.";
}

}  // namespace autoware::behavior_path_planner
