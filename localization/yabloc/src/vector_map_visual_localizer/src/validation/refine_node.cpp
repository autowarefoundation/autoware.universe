#include "common/util.hpp"
#include "validation/refine.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace validation
{
RefineOptimizer::RefineOptimizer() : Overlay("refine_opt") {}

void RefineOptimizer::imageCallback(const Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
  const rclcpp::Time stamp = msg.header.stamp;

  // Search synchronized pose
  float min_dt = std::numeric_limits<float>::max();
  geometry_msgs::msg::PoseStamped synched_pose;
  for (auto pose : pose_buffer_) {
    auto dt = (rclcpp::Time(pose.header.stamp) - stamp);
    auto abs_dt = std::abs(dt.seconds());
    if (abs_dt < min_dt) {
      min_dt = abs_dt;
      synched_pose = pose;
    }
  }
  if (min_dt > 0.1) return;
  auto latest_pose_stamp = rclcpp::Time(pose_buffer_.back().header.stamp);
  RCLCPP_INFO_STREAM(
    get_logger(), "dt: " << min_dt << " image:" << stamp.nanoseconds()
                         << " latest_pose:" << latest_pose_stamp.nanoseconds());

  drawOverlay(image, synched_pose.pose, stamp);
}

}  // namespace validation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::RefineOptimizer>());
  rclcpp::shutdown();
  return 0;
}