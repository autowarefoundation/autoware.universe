#include "vml_common/util.hpp"

namespace vml_common
{
Eigen::Affine3f pose2Affine(const geometry_msgs::msg::Pose & pose)
{
  const auto pos = pose.position;
  const auto ori = pose.orientation;
  Eigen::Translation3f t(pos.x, pos.y, pos.z);
  Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  return t * q;
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
  if (t->tm_hour >= 9) {
    msg.hour = t->tm_hour - 9;
  } else {
    msg.hour = t->tm_hour + 15;
    msg.day--;
  }
  msg.min = t->tm_min;
  msg.sec = t->tm_sec;
  msg.nano = stamp.nanosec;
  return msg;
}
}  // namespace vml_common
