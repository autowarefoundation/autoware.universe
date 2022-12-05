#include "hsv_extractor/hsv_extractor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::hsv_extractor::HsvExtractor>());
  rclcpp::shutdown();
  return 0;
}