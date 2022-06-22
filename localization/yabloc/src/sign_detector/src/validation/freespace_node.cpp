#include "validation/freespace.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::FreeSpace>());
  rclcpp::shutdown();
  return 0;
}