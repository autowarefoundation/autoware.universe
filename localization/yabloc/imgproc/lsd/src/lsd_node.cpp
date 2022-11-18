#include "lsd/lsd.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::lsd::LineSegmentDetector>());
  rclcpp::shutdown();
  return 0;
}