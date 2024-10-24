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

#include "autoware/costmap_generator/costmap_generator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

using autoware::costmap_generator::CostmapGeneratorNode;

class TestCostmapGenerator : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    set_up_node();
    setup_lanelet_map();
  }

  void set_up_node()
  {
    auto node_options = rclcpp::NodeOptions{};
    const auto costmap_generator_dir =
      ament_index_cpp::get_package_share_directory("autoware_costmap_generator");
    node_options.arguments(
      {"--ros-args", "--params-file",
       costmap_generator_dir + "/config/costmap_generator.param.yaml"});
    costmap_generator_ = std::make_shared<CostmapGeneratorNode>(node_options);
  }

  void setup_lanelet_map()
  {
    const auto lanelet2_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_test_utils", "overlap_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet2_path, 5.0);
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(map_bin_msg, lanelet_map_);
  }

  [[nodiscard]] double get_grid_length_x() { return costmap_generator_->costmap_.getLength()[0]; }

  [[nodiscard]] double get_grid_length_y() { return costmap_generator_->costmap_.getLength()[1]; }

  [[nodiscard]] double get_grid_resolution()
  {
    return costmap_generator_->costmap_.getResolution();
  }

  [[nodiscard]] std::vector<std::string> get_grid_layers()
  {
    return costmap_generator_->costmap_.getLayers();
  }

  size_t test_load_road_areas()
  {
    std::vector<std::vector<geometry_msgs::msg::Point>> road_areas;
    costmap_generator_->loadRoadAreasFromLaneletMap(lanelet_map_, &road_areas);
    return road_areas.size();
  }

  size_t test_load_parking_areas()
  {
    std::vector<std::vector<geometry_msgs::msg::Point>> parking_areas;
    costmap_generator_->loadParkingAreasFromLaneletMap(lanelet_map_, &parking_areas);
    return parking_areas.size();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    costmap_generator_ = nullptr;
    lanelet_map_ = nullptr;
  }

  std::shared_ptr<CostmapGeneratorNode> costmap_generator_;
  lanelet::LaneletMapPtr lanelet_map_;
};

TEST_F(TestCostmapGenerator, testInitializeGridmap)
{
  const double grid_resolution = get_grid_resolution();
  const double grid_length_x = get_grid_length_x();
  const double grid_length_y = get_grid_length_y();
  EXPECT_FLOAT_EQ(grid_resolution, 0.3);
  EXPECT_TRUE(70.0 - grid_length_x < grid_resolution);
  EXPECT_TRUE(70.0 - grid_length_y < grid_resolution);

  const auto grid_layers = get_grid_layers();

  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "points") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "objects") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "primitives") != grid_layers.end());
  EXPECT_TRUE(std::find(grid_layers.begin(), grid_layers.end(), "combined") != grid_layers.end());
}

TEST_F(TestCostmapGenerator, testLoadAreasFromLaneletMap)
{
  EXPECT_EQ(test_load_road_areas(), 12ul);
  EXPECT_EQ(test_load_parking_areas(), 10ul);
}
