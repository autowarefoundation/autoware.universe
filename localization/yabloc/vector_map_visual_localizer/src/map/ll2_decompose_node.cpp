#include "map/ll2_decomposer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map::Ll2Decomposer>());
  rclcpp::shutdown();
  return 0;
}
