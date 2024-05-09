// Copyright 2024 TIER IV
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

#ifndef TEST_ROUTE_HANDLER_HPP_
#define TEST_ROUTE_HANDLER_HPP_

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"

#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <lanelet2_io/Io.h>

#include <memory>
#include <string>
namespace route_handler::test
{

using autoware_auto_mapping_msgs::msg::HADMapBin;

class TestRouteHandler : public ::testing::Test
{
public:
  TestRouteHandler() { route_handler_ = std::make_shared<RouteHandler>(makeMapBinMsg()); }
  TestRouteHandler(const TestRouteHandler &) = delete;
  TestRouteHandler(TestRouteHandler &&) = delete;
  TestRouteHandler & operator=(const TestRouteHandler &) = delete;
  TestRouteHandler & operator=(TestRouteHandler &&) = delete;
  ~TestRouteHandler() override = default;

  static lanelet::LaneletMapPtr loadMap(const std::string & lanelet2_filename)
  {
    lanelet::ErrorMessages errors{};
    lanelet::projection::MGRSProjector projector{};
    auto map = lanelet::load(lanelet2_filename, projector, &errors);
    if (errors.empty()) {
      return map;
    }

    for (const auto & error : errors) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
    }
    return nullptr;
  }

  static HADMapBin makeMapBinMsg()
  {
    const auto planning_test_utils_dir =
      ament_index_cpp::get_package_share_directory("route_handler");
    const auto lanelet2_path = planning_test_utils_dir + "/test_map/lanelet2_map.osm";
    double center_line_resolution = 5.0;
    // load map from file
    const auto map = loadMap(lanelet2_path);
    if (!map) {
      return autoware_auto_mapping_msgs::msg::HADMapBin_<std::allocator<void>>{};
    }

    // overwrite centerline
    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

    // create map bin msg
    const auto map_bin_msg =
      convertToMapBinMsg(map, lanelet2_path, rclcpp::Clock(RCL_ROS_TIME).now());
    return map_bin_msg;
  }

  static HADMapBin convertToMapBinMsg(
    const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename,
    const rclcpp::Time & now)
  {
    std::string format_version{};
    std::string map_version{};
    lanelet::io_handlers::AutowareOsmParser::parseVersions(
      lanelet2_filename, &format_version, &map_version);

    HADMapBin map_bin_msg;
    map_bin_msg.header.stamp = now;
    map_bin_msg.header.frame_id = "map";
    map_bin_msg.format_version = format_version;
    map_bin_msg.map_version = map_version;
    lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

    return map_bin_msg;
  }

  std::shared_ptr<RouteHandler> route_handler_;
};
}  // namespace route_handler::test

#endif  // TEST_ROUTE_HANDLER_HPP_
