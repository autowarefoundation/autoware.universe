#include "graph_segment/graph_segment.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::graph_segment::GraphSegment>());
  rclcpp::shutdown();
  return 0;
}