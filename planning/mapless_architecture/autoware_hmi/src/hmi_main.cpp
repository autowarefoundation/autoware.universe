// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license

#include "autoware/hmi/hmi_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("hmi"), "Launching HMI node...");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::mapless_architecture::HMINode>());
  rclcpp::shutdown();
  return 0;
}
