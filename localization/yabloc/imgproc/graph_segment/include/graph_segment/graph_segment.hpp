#pragma once
#include <opencv4/opencv2/ximgproc/segmentation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcdless::graph_segment
{
class GraphSegment : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  GraphSegment();

private:
  const float target_height_ratio_;
  const int target_candidate_box_width_;

  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation> segmentation_;

  void on_image(const Image & msg);
  void execute(const PointCloud2 & msg1, const PointCloud2 & msg2);
  void publish_image(
    const cv::Mat & raw_image, const cv::Mat & segmentation, const rclcpp::Time & stamp,
    int target_class);
};
}  // namespace pcdless::graph_segment