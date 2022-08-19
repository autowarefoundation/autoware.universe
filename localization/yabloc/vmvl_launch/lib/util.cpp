#include "common/util.hpp"

#include <opencv4/opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

namespace util
{

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

ublox_msgs::msg::NavPVT stamp2UbloxTime(const builtin_interfaces::msg::Time & stamp)
{
  time_t t_of_day = stamp.sec;
  ublox_msgs::msg::NavPVT msg;
  struct tm * t = localtime(&t_of_day);
  msg.year = t->tm_year + 1900;
  msg.month = t->tm_mon + 1;
  msg.day = t->tm_mday;
  msg.hour = t->tm_hour - 9;
  if (msg.hour < 0) {
    msg.hour += 24;
    msg.day--;
  }
  msg.min = t->tm_min;
  msg.sec = t->tm_sec;
  msg.nano = stamp.nanosec;
  return msg;
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
