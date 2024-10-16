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

#include "autoware/behavior_path_lane_change_module/manager.hpp"
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <memory>

using autoware::behavior_path_planner::LaneChangeModuleManager;
using autoware::behavior_path_planner::LaneChangeModuleType;
using autoware::behavior_path_planner::NormalLaneChange;
using autoware::behavior_path_planner::PlannerData;
using autoware::behavior_path_planner::lane_change::LCParamPtr;
using autoware::route_handler::Direction;
using autoware::route_handler::RouteHandler;
using autoware::test_utils::get_absolute_path_to_lanelet_map;
using autoware::test_utils::get_absolute_path_to_route;
using autoware_map_msgs::msg::LaneletMapBin;

class TestNormalLaneChange : public ::testing::Test
{
public:
  void SetUp() override
  {
    init_param();
    init_module();
  }

  void init_param() const
  {
    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto behavior_path_lane_change_module_dir =
      ament_index_cpp::get_package_share_directory("autoware_behavior_path_lane_change_module");
    const auto behavior_path_planner_dir =
      ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner");

    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_common.param.yaml",
                     autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
                     autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                     behavior_path_planner_dir + "/config/behavior_path_planner.param.yaml",
                     behavior_path_planner_dir + "/config/drivable_area_expansion.param.yaml",
                     behavior_path_planner_dir + "/config/scene_module_manager.param.yaml",
                     behavior_path_lane_change_module_dir + "/config/lane_change.param.yaml"});

    auto node = rclcpp::Node(name, node_options);
    planner_data_->init_parameters(node);
    auto lc_param = LaneChangeModuleManager::set_params(&node, node.get_name());

    std::string autoware_route_handler_dir{"autoware_route_handler"};
    std::string lane_change_right_test_route_filename{"lane_change_test_route.yaml"};
    std::string lanelet_map_filename{"2km_test.osm"};
    const auto lanelet2_path =
      get_absolute_path_to_lanelet_map(autoware_test_utils_dir, lanelet_map_filename);
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet2_path, 5.0);
    planner_data_->route_handler = std::make_shared<RouteHandler>(map_bin_msg);
    const auto rh_test_route =
      get_absolute_path_to_route(autoware_route_handler_dir, lane_change_right_test_route_filename);
    planner_data_->route_handler->setRoute(
      autoware::test_utils::parse_lanelet_route_file(rh_test_route));
  }

  void init_module()
  {
    normal_lane_change_ =
      std::make_shared<NormalLaneChange>(lc_param_ptr_, lc_type_, lc_direction_);
    normal_lane_change_->setData(planner_data_);
  }

  LCParamPtr lc_param_ptr_;
  std::shared_ptr<NormalLaneChange> normal_lane_change_;
  std::shared_ptr<PlannerData> planner_data_ = std::make_shared<PlannerData>();
  LaneChangeModuleType lc_type_{LaneChangeModuleType::NORMAL};
  Direction lc_direction_{Direction::RIGHT};
  std::string name = "test_lane_change_scene";

  autoware::behavior_path_planner::lane_change::CommonDataPtr get_common_data_ptr()
  {
    return normal_lane_change_->common_data_ptr_;
  }

  void TearDown() override
  {
    normal_lane_change_ = nullptr;
    lc_param_ptr_ = nullptr;
    planner_data_ = nullptr;
    rclcpp::shutdown();
  }
};

TEST_F(TestNormalLaneChange, testBaseClassInitialize)
{
  const auto type = normal_lane_change_->getModuleType();
  const auto type_str = normal_lane_change_->getModuleTypeStr();

  ASSERT_EQ(type, LaneChangeModuleType::NORMAL);
  ASSERT_TRUE(type_str == "NORMAL");

  ASSERT_EQ(normal_lane_change_->getDirection(), Direction::RIGHT);

  ASSERT_TRUE(get_common_data_ptr());

  ASSERT_FALSE(get_common_data_ptr()->is_data_available());
}
