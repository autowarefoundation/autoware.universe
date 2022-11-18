#include "segment_accumulator/segment_accumulator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::accumulator::SegmentAccumulator>());
  rclcpp::shutdown();
  return 0;
}
