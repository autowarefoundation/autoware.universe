#include "sign_detector/lsd.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineSegmentDetector>());
  rclcpp::shutdown();
  return 0;
}