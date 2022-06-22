#include "validation/overlay.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::Overlay>());
  rclcpp::shutdown();
  return 0;
}