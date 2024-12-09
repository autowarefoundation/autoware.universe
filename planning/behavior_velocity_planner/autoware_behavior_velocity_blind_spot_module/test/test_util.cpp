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

#include <autoware/behavior_velocity_blind_spot_module/parameter.hpp>
#include <autoware/behavior_velocity_blind_spot_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef EXPORT_TEST_PLOT_FIGURE
#include <autoware/pyplot/common.hpp>
#include <autoware/pyplot/patches.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_test_utils/visualization.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>  // needed for passing std::string to Args

#endif

using autoware::test_utils::parse;

class TestWithAdjLaneData : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_data_file =
      ament_index_cpp::get_package_share_directory("autoware_behavior_velocity_blind_spot_module") +
      "/test_data/object_on_adj_lane.yaml";
    const auto config = YAML::LoadFile(test_data_file);
    const auto route = parse<autoware_planning_msgs::msg::LaneletRoute>(config["route"]);
    const auto map_path =
      autoware::test_utils::resolve_pkg_share_uri(config["map_path_uri"].as<std::string>());
    if (!map_path) {
      ASSERT_DEATH({ assert(false); }, "invalid map path");
    }
    const auto intersection_map = autoware::test_utils::make_map_bin_msg(map_path.value());
    route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
    route_handler->setMap(intersection_map);
    route_handler->setRoute(route);
    self_odometry = autoware::test_utils::create_const_shared_ptr(
      parse<nav_msgs::msg::Odometry>(config["self_odometry"]));
    dynamic_object = autoware::test_utils::create_const_shared_ptr(
      parse<autoware_perception_msgs::msg::PredictedObjects>(config["dynamic_object"]));

    // parameter
    auto node_options = rclcpp::NodeOptions{};
    node_options.arguments(std::vector<std::string>{
      "--ros-args",
      "--params-file",
      ament_index_cpp::get_package_share_directory("autoware_behavior_velocity_blind_spot_module") +
        "/config/blind_spot.param.yaml",
    });

    auto node = rclcpp::Node::make_shared("blind_spot_test", node_options);
    param = autoware::behavior_velocity_planner::PlannerParam::init(*node, "blind_spot");
  }

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler{};
  std::shared_ptr<const nav_msgs::msg::Odometry> self_odometry{};
  std::shared_ptr<const autoware_perception_msgs::msg::PredictedObjects> dynamic_object{};
  const lanelet::Id lane_id_{2200};
  autoware::behavior_velocity_planner::PlannerParam param;
};

TEST_F(TestWithAdjLaneData, getSiblingStraightLanelet)
{
  const auto sibling_straight_lanelet_opt =
    autoware::behavior_velocity_planner::getSiblingStraightLanelet(
      route_handler->getLaneletMapPtr()->laneletLayer.get(lane_id_),
      route_handler->getRoutingGraphPtr());
  ASSERT_NO_FATAL_FAILURE({ ASSERT_TRUE(sibling_straight_lanelet_opt.has_value()); });
  EXPECT_EQ(sibling_straight_lanelet_opt.value().id(), 2100);

#ifdef EXPORT_TEST_PLOT_FIGURE
  py::gil_scoped_acquire acquire;
  using autoware::test_utils::LaneConfig;
  using autoware::test_utils::LineConfig;
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto & ax = axes[0];
  autoware::test_utils::plot_lanelet2_object(
    route_handler->getLaneletMapPtr()->laneletLayer.get(lane_id_), ax,
    autoware::test_utils::LaneConfig{"original", LineConfig{"blue"}});

  // for illustration
  autoware::test_utils::plot_lanelet2_object(
    route_handler->getLaneletMapPtr()->laneletLayer.get(3010933), ax,
    autoware::test_utils::LaneConfig{std::nullopt, LineConfig{"green"}});
  autoware::test_utils::plot_lanelet2_object(
    route_handler->getLaneletMapPtr()->laneletLayer.get(2201), ax,
    autoware::test_utils::LaneConfig{std::nullopt, LineConfig{"blue"}});
  autoware::test_utils::plot_lanelet2_object(
    route_handler->getLaneletMapPtr()->laneletLayer.get(2202), ax,
    autoware::test_utils::LaneConfig{std::nullopt, LineConfig{"blue"}});
  autoware::test_utils::plot_lanelet2_object(
    route_handler->getLaneletMapPtr()->laneletLayer.get(2010), ax,
    autoware::test_utils::LaneConfig{std::nullopt, LineConfig{"green"}});

  autoware::test_utils::plot_lanelet2_object(
    sibling_straight_lanelet_opt.value(), ax,
    LaneConfig{"sibling_straight_lanelet", LineConfig{"red"}});
  const std::string filename = std::string(
    ament_index_cpp::get_package_share_directory("autoware_behavior_velocity_blind_spot_module") +
    "/test_data/getSiblingStraightLaneletTest.svg");
  ax.set_aspect(Args("equal"));
  ax.grid();
  plt.savefig(Args(filename));
#endif
}

TEST_F(TestWithAdjLaneData, generateHalfLanelet)
{
  const auto lanelet = route_handler->getLaneletMapPtr()->laneletLayer.get(2010);

  const auto half_lanelet = autoware::behavior_velocity_planner::generateHalfLanelet(
    lanelet, autoware::behavior_velocity_planner::TurnDirection::LEFT,
    param.ignore_width_from_center_line);

  /*
    TODO(soblin): how to check if they overlap only on the left line string
  EXPECT_EQ(
    boost::geometry::within(
      half_lanelet.polygon2d().basicPolygon(), lanelet.polygon2d().basicPolygon()),
    true);
  */
  EXPECT_EQ(
    boost::geometry::area(lanelet.polygon2d().basicPolygon()) / 2.0 >
      boost::geometry::area(half_lanelet.polygon2d().basicPolygon()),
    true);

#ifdef EXPORT_TEST_PLOT_FIGURE
  py::gil_scoped_acquire acquire;
  using autoware::test_utils::LaneConfig;
  using autoware::test_utils::LineConfig;
  auto plt = autoware::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto & ax = axes[0];
  autoware::test_utils::plot_lanelet2_object(
    lanelet, ax, autoware::test_utils::LaneConfig{"original", LineConfig{"blue", 1.5}});
  autoware::test_utils::plot_lanelet2_object(
    half_lanelet, ax, LaneConfig{"half_lanelet", LineConfig{"red", 0.75}});
  const std::string filename = std::string(
    ament_index_cpp::get_package_share_directory("autoware_behavior_velocity_blind_spot_module") +
    "/test_data/generateHalfLaneletTest.svg");
  ax.set_aspect(Args("equal"));
  ax.grid();
  plt.savefig(Args(filename));
#endif
}

int main(int argc, char ** argv)
{
  py::scoped_interpreter guard{};
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
