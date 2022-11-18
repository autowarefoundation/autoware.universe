#include "pcdless_common/pub_sub.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcdless::common
{
void publish_image(
  rclcpp::Publisher<sensor_msgs::msg::Image> & publisher, const cv::Mat & image,
  const rclcpp::Time & stamp)
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  raw_image.encoding = "bgr8";
  raw_image.image = image;
  publisher.publish(*raw_image.toImageMsg());
}

template <typename PointT>
void publish_cloud(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & publisher,
  const pcl::PointCloud<PointT> & cloud, const rclcpp::Time & stamp)
{
  // Convert to msg
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "map";
  publisher.publish(cloud_msg);
}

template void publish_cloud<pcl::PointXYZ>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZ> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointNormal>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointNormal> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointXYZI>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZI> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointXYZRGB>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZRGB> &,
  const rclcpp::Time &);

}  // namespace pcdless::common