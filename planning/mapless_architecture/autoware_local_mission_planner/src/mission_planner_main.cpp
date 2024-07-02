// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#include "autoware/local_mission_planner/mission_planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("mission_planner"), "Launching mission planner node...");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::mapless_architecture::MissionPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
