#pragma once
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vml_common/base_camera_info_node.hpp>
#include <vml_common/static_tf_subscriber.hpp>
#include <vml_common/synchro_subscriber.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

namespace vmvl_imgproc
{
class SegmentFilter : public vml_common::BaseCameraInfoNode
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;
  SegmentFilter();

private:
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const Eigen::Vector3f &)>;
  const int image_size_;
  const float max_range_;
  const int truncate_pixel_threshold_;
  const float min_segment_length_;
  const float max_segment_distance_;

  SynchroSubscriber<PointCloud2, PointCloud2> subscriber_;
  vml_common::StaticTfSubscriber tf_subscriber_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  pcl::PointCloud<pcl::PointNormal> projectLines(
    const pcl::PointCloud<pcl::PointNormal> & lines, ProjectFunc project) const;
  pcl::PointCloud<pcl::PointXYZ> projectMask(
    const pcl::PointCloud<pcl::PointXYZ> & mask, ProjectFunc project) const;

  pcl::PointIndices filtByMask(
    const pcl::PointCloud<pcl::PointXYZ> & mask, const pcl::PointCloud<pcl::PointNormal> & edges);

  cv::Point2i toCvPoint(const Eigen::Vector3f & v) const;
  void execute(const PointCloud2 & msg1, const PointCloud2 & msg2);

  // TODO: Rename function
  bool isLowerElement(const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const;
  bool isNearElement(const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const;
};
}  // namespace vmvl_imgproc