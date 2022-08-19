#include "vmvl_validation/freespace.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vmvl_validation::FreeSpace>());
  rclcpp::shutdown();
  return 0;
}