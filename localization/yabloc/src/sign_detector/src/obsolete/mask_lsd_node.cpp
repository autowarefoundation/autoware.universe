#include "sign_detector/mask_lsd.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaskLsd>());
  rclcpp::shutdown();
  return 0;
}