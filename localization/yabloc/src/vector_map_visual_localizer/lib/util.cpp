#include "common/util.hpp"

#include <opencv4/opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

namespace util
{

cv_bridge::CvImage decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img)
{
  cv_bridge::CvImage raw_image;
  raw_image.header = compressed_img.header;

  const std::string & format = compressed_img.format;
  const std::string encoding = format.substr(0, format.find(";"));
  raw_image.encoding = encoding;

  constexpr int DECODE_GRAY = 0;
  constexpr int DECODE_RGB = 1;

  bool encoding_is_bayer = encoding.find("bayer") != std::string::npos;
  if (!encoding_is_bayer) {
    raw_image.image = cv::imdecode(cv::Mat(compressed_img.data), DECODE_RGB);
    return raw_image;
  }

  raw_image.image = cv::imdecode(cv::Mat(compressed_img.data), DECODE_GRAY);
  if (encoding == "bayer_rggb8")
    cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerBG2BGR);
  else if (encoding == "bayer_bggr8")
    cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerRG2BGR);
  else if (encoding == "bayer_grbg8")
    cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerGB2BGR);
  else if (encoding == "bayer_gbrg8")
    cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerGR2BGR);
  else {
    std::cerr << encoding << " is not supported encoding" << std::endl;
    std::cerr << "Please implement additional decoding in " << __FUNCTION__ << std::endl;
    exit(EXIT_FAILURE);
  }
  raw_image.encoding = "bgr8";
  return raw_image;
}

cv::Mat decompress2CvMat(const sensor_msgs::msg::CompressedImage & compressed_img)
{
  sensor_msgs::msg::Image::ConstSharedPtr msg_ptr = decompress2RosMsg(compressed_img);
  return cv_bridge::toCvCopy(msg_ptr, msg_ptr->encoding)->image;
}

cv::Mat decompress2CvMat(const sensor_msgs::msg::Image & img)
{
  return cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(img), img.encoding)->image;
}

sensor_msgs::msg::Image::ConstSharedPtr decompress2RosMsg(
  const sensor_msgs::msg::CompressedImage & compressed_img)
{
  cv_bridge::CvImage cv_image = decompressImage(compressed_img);
  return cv_image.toImageMsg();
}

void publishImage(
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

geometry_msgs::msg::Pose affine2Pose(const Eigen::Affine3f & affine)
{
  geometry_msgs::msg::Pose pose;
  Eigen::Vector3f pos = affine.translation();
  Eigen::Quaternionf ori(affine.rotation());
  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = pos.z();
  pose.orientation.w = ori.w();
  pose.orientation.x = ori.x();
  pose.orientation.y = ori.y();
  pose.orientation.z = ori.z();
  return pose;
}

Eigen::Affine3f pose2Affine(const geometry_msgs::msg::Pose & pose)
{
  const auto pos = pose.position;
  const auto ori = pose.orientation;
  Eigen::Translation3f t(pos.x, pos.y, pos.z);
  Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  return t * q;
}

#include <time.h>
rclcpp::Time ubloxTime2Stamp(const ublox_msgs::msg::NavPVT & msg)
{
  // TODO: Day 31 may cause invalid convertion
  struct tm t;
  t.tm_year = msg.year - 1900;  // from 1900
  t.tm_mon = msg.month - 1;     // january = 0
  t.tm_mday = msg.day;
  t.tm_hour = msg.hour + 9;  // JST is +9
  if (t.tm_hour > 23) {
    t.tm_mday++;
    t.tm_hour -= 24;
  }
  t.tm_min = msg.min;
  t.tm_sec = msg.sec;
  t.tm_isdst = 0;

  time_t t_of_day = mktime(&t);

  uint32_t nano = 0;
  if (msg.nano >= 0) {
    nano = msg.nano;
  } else {
    t_of_day--;
    nano = 1e9 + msg.nano;
  }

  rclcpp::Time stamp(t_of_day, nano, RCL_ROS_TIME);
  return stamp;
}

template <typename PointT>
void publishCloud(
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

template void publishCloud<pcl::PointXYZ>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZ> &,
  const rclcpp::Time &);

template void publishCloud<pcl::PointNormal>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointNormal> &,
  const rclcpp::Time &);

template void publishCloud<pcl::PointXYZI>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZI> &,
  const rclcpp::Time &);

}  // namespace util
