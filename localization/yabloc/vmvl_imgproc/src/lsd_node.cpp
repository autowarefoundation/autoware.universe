#include "vmvl_imgproc/lsd.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::LineSegmentDetector>());
  rclcpp::shutdown();
  return 0;
}