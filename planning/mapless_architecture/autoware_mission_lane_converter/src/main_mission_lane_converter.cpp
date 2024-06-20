// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#include "autoware/mission_lane_converter/mission_lane_converter_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  RCLCPP_INFO(
    rclcpp::get_logger("mission_planner_converter_node"),
    "Launching mission lane converter node...");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::mapless_architecture::MissionLaneConverterNode>());
  rclcpp::shutdown();
  return 0;
}
