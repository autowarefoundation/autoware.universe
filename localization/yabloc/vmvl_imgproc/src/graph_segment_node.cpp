#include "vmvl_imgproc/graph_segment.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::GraphSegment>());
  rclcpp::shutdown();
  return 0;
}