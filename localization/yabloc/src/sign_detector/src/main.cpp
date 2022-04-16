#include "sign_detector/lsd.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>("/sensing/camera/traffic_light/image_raw/compressed"));
  rclcpp::shutdown();
  return 0;
}