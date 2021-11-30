/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <autoware_lanelet2_msgs/msg/map_bin.hpp>

#include <string>

void printUsage(const rclcpp::Logger & logger)
{
  RCLCPP_ERROR_STREAM(
    logger,
    "Usage:" << std::endl
             << "rosrun map_loader lanelet2_map_loader [.OSM]" << std::endl
             << "rosrun map_loader lanelet2_map_loader download [X] [Y]: WARNING not implemented");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lanelet_map_loader");

  if (argc < 2) {
    printUsage(node->get_logger());
    return EXIT_FAILURE;
  }

  std::string mode(argv[1]);
  if (mode == "download" && argc < 4) {
    printUsage(node->get_logger());
    return EXIT_FAILURE;
  }

  std::string lanelet2_filename(argv[1]);

  lanelet::ErrorMessages errors;

  lanelet::projection::MGRSProjector projector;
  lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(node->get_logger(), error);
  }
  if (!errors.empty()) {
    return EXIT_FAILURE;
  }

  double center_line_resolution;
  center_line_resolution = node->declare_parameter("center_line_resolution", 5.0);
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  std::string format_version, map_version;
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  const auto map_bin_pub =
    node->create_publisher<autoware_lanelet2_msgs::msg::MapBin>("output/lanelet2_map", durable_qos);
  autoware_lanelet2_msgs::msg::MapBin map_bin_msg;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  map_bin_msg.header.stamp = ros_clock.now();
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  map_bin_pub->publish(map_bin_msg);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
