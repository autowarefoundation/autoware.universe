#include "ll2_decomposer/ll2_decomposer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::ll2_decomposer::Ll2Decomposer>());
  rclcpp::shutdown();
  return 0;
}
