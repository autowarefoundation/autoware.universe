#include "sign_detector/ll2_to_image.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ll2ImageConverter>());
  rclcpp::shutdown();
  return 0;
}
