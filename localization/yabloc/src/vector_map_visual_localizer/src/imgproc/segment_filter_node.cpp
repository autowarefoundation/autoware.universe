#include "imgproc/segment_filter.hpp"

namespace imgproc
{
SegmentFilter::SegmentFilter()
: Node("segment_filter"), subscriber_(rclcpp::Node::SharedPtr{this}, "lsd_cloud", "graph_segmented")
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  auto cb = std::bind(&SegmentFilter::execute, this, _1, _2);
  subscriber_.setCallback(cb);
}

void SegmentFilter::execute(const PointCloud2 & msg1, const PointCloud2 & msg2)
{
  RCLCPP_INFO_STREAM(get_logger(), "synchro subscriber works well");
}

}  // namespace imgproc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::SegmentFilter>());
  rclcpp::shutdown();
  return 0;
}
