#include "map/ground_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map::GroundServer>());
  rclcpp::shutdown();
}