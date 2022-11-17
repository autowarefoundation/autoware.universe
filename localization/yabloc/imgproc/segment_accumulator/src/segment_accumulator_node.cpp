#include "segment_accumulator/segment_accumulator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::Mapping>());
  rclcpp::shutdown();
  return 0;
}
