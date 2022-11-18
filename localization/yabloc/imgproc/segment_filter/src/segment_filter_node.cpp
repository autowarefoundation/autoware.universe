#include "segment_filter/segment_filter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::segment_filter::SegmentFilter>());
  rclcpp::shutdown();
  return 0;
}
