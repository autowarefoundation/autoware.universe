#include "anti_shadow/reproject.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::Reprojector>());
  rclcpp::shutdown();
  return 0;
}