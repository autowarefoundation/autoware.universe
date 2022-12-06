#pragma once
#include <opencv4/opencv2/core.hpp>
#include <pcdless_common/camera_info_subscriber.hpp>
#include <pcdless_common/static_tf_subscriber.hpp>
#include <pcdless_common/synchro_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::segment_filter
{
class SegmentFilter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  SegmentFilter();

private:
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const Eigen::Vector3f &)>;
  const int image_size_;
  const float max_range_;
  const float min_segment_length_;
  const float max_segment_distance_;
  const float max_lateral_distance_;

  common::CameraInfoSubscriber info_;
  common::SynchroSubscriber<PointCloud2, Image> synchro_subscriber_;
  common::StaticTfSubscriber tf_subscriber_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  pcl::PointCloud<pcl::PointNormal> project_lines(
    const pcl::PointCloud<pcl::PointNormal> & lines, ProjectFunc project,
    const std::set<int> & indices, bool negative = false) const;

  std::set<int> filt_by_mask(const cv::Mat & mask, const pcl::PointCloud<pcl::PointNormal> & edges);

  cv::Point2i to_cv_point(const Eigen::Vector3f & v) const;
  void execute(const PointCloud2 & msg1, const Image & msg2);

  // TODO: Rename function
  bool is_near_element(const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const;
};
}  // namespace pcdless::segment_filter