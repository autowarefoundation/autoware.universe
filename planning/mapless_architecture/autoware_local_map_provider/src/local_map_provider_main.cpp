// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license

#include "autoware/local_map_provider/local_map_provider_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("local_map_provider"), "Launching Local Map Provider node...");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::mapless_architecture::LocalMapProviderNode>());
  rclcpp::shutdown();
  return 0;
}
