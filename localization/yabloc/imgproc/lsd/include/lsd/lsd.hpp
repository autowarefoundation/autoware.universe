#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace pcdless::lsd
{
class LineSegmentDetector : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  LineSegmentDetector();

private:
  const int truncate_pixel_threshold_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_lsd_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;

  cv::Ptr<cv::LineSegmentDetector> lsd_;

  std::vector<cv::Mat> remove_too_outer_elements(
    const cv::Mat & lines, const cv::Size & size) const;
  void on_image(const sensor_msgs::msg::Image & msg);
  void execute(const cv::Mat & image, const rclcpp::Time & stamp);
};
}  // namespace pcdless::lsd