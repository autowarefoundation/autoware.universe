/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "gnss_poser/gnss_poser_core.h"

#include <algorithm>
#include <string>

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <ublox_msgs/NavPVT.h>

#include "gnss_poser/convert.h"

namespace GNSSPoser
{
GNSSPoser::GNSSPoser(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh),
  private_nh_(private_nh),
  tf2_listener_(tf2_buffer_),
  coordinate_system_(CoordinateSystem::MGRS),
  base_frame_("base_link"),
  gnss_frame_("gnss"),
  gnss_base_frame_("gnss_base_link"),
  map_frame_("map"),
  use_ublox_receiver_(false),
  plane_zone_(9)
{
  int coordinate_system = static_cast<int>(coordinate_system_);
  private_nh_.getParam("coordinate_system", coordinate_system);
  coordinate_system_ = static_cast<CoordinateSystem>(coordinate_system);

  private_nh_.getParam("base_frame", base_frame_);
  private_nh_.getParam("gnss_frame", gnss_frame_);
  private_nh_.getParam("gnss_base_frame", gnss_base_frame_);
  private_nh_.getParam("map_frame", map_frame_);
  private_nh_.getParam("use_ublox_receiver", use_ublox_receiver_);
  private_nh_.getParam("plane_zone", plane_zone_);

  int buff_epoch = 1;
  private_nh_.getParam("buff_epoch", buff_epoch);
  position_buffer_.set_capacity(buff_epoch);

  nav_sta_fix_sub_ = nh_.subscribe("fix", 10, &GNSSPoser::callbackNavSatFix, this);
  nav_pvt_sub_ = nh_.subscribe("navpvt", 10, &GNSSPoser::callbackNavPVT, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
  pose_cov_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("gnss_pose_cov", 10);
  fixed_pub_ = nh_.advertise<std_msgs::Bool>("gnss_fixed", 10);
}

void GNSSPoser::callbackNavSatFix(const sensor_msgs::NavSatFix::ConstPtr & nav_sat_fix_msg_ptr)
{
  // check fixed topic
  const bool is_fixed = isFixed(nav_sat_fix_msg_ptr->status);

  // publish is_fixed topic
  std_msgs::Bool is_fixed_msg;
  is_fixed_msg.data = is_fixed;
  fixed_pub_.publish(is_fixed_msg);

  if (!is_fixed) {
    ROS_WARN_STREAM_THROTTLE(1, "Not Fixed Topic. Skipping Calculate.");
    return;
  }

  // get position in coordinate_system
  const auto gnss_stat = convert(*nav_sat_fix_msg_ptr, coordinate_system_);
  const auto position = getPosition(gnss_stat);

  // calc median position
  position_buffer_.push_front(position);
  if (!position_buffer_.full()) {
    ROS_WARN_STREAM_THROTTLE(1, "Buffering Position. Output Skipped.");
    return;
  }
  const auto median_position = getMedianPosition(position_buffer_);

  // calc gnss antenna orientation
  geometry_msgs::Quaternion orientation;
  if (use_ublox_receiver_ && nav_pvt_msg_ptr_ != nullptr) {
    orientation = getQuaternionByHeading(nav_pvt_msg_ptr_->heading);
  } else {
    static auto prev_position = median_position;
    orientation = getQuaternionByPositionDiffence(median_position, prev_position);
    prev_position = median_position;
  }

  // generate gnss_antenna_pose_msg
  geometry_msgs::PoseStamped gnss_antenna_pose_msg;
  gnss_antenna_pose_msg.header.stamp = nav_sat_fix_msg_ptr->header.stamp;
  gnss_antenna_pose_msg.header.frame_id = map_frame_;
  gnss_antenna_pose_msg.pose.position = median_position;
  gnss_antenna_pose_msg.pose.orientation = orientation;

  // get TF from base_link to gnss_antenna
  geometry_msgs::TransformStamped::Ptr TF_base_to_gnss_ptr(new geometry_msgs::TransformStamped);
  getStaticTransform(gnss_frame_, base_frame_, TF_base_to_gnss_ptr, nav_sat_fix_msg_ptr->header.stamp);

  // transform pose from gnss_antenna to base_link
  geometry_msgs::PoseStamped gnss_base_pose_msg;
  //remove rotation
  TF_base_to_gnss_ptr->transform.rotation.x = 0.0;
  TF_base_to_gnss_ptr->transform.rotation.y = 0.0;
  TF_base_to_gnss_ptr->transform.rotation.z = 0.0;
  TF_base_to_gnss_ptr->transform.rotation.w = 1.0;
  tf2::doTransform(gnss_antenna_pose_msg, gnss_base_pose_msg, *TF_base_to_gnss_ptr);
  gnss_base_pose_msg.header.frame_id = map_frame_;

  // publish gnss_base_link pose in map frame
  pose_pub_.publish(gnss_base_pose_msg);

  // publish gnss_base_link pose_cov in map frame
  geometry_msgs::PoseWithCovarianceStamped gnss_base_pose_cov_msg;
  gnss_base_pose_cov_msg.header = gnss_base_pose_msg.header;
  gnss_base_pose_cov_msg.pose.pose = gnss_base_pose_msg.pose;
  gnss_base_pose_cov_msg.pose.covariance[6 * 0 + 0] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[0] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[6 * 1 + 1] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[4] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[6 * 2 + 2] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[8] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[6 * 3 + 3] = 0.1;
  gnss_base_pose_cov_msg.pose.covariance[6 * 4 + 4] = 0.1;
  gnss_base_pose_cov_msg.pose.covariance[6 * 5 + 5] = 1.0;
  pose_cov_pub_.publish(gnss_base_pose_cov_msg);

  // broadcast map to gnss_base_link
  publishTF(map_frame_, gnss_base_frame_, gnss_base_pose_msg);
}

void GNSSPoser::callbackNavPVT(const ublox_msgs::NavPVT::ConstPtr & nav_pvt_msg_ptr)
{
  nav_pvt_msg_ptr_ = nav_pvt_msg_ptr;
}

bool GNSSPoser::isFixed(const sensor_msgs::NavSatStatus & nav_sat_status_msg)
{
  return nav_sat_status_msg.status >= sensor_msgs::NavSatStatus::STATUS_FIX;
}

bool GNSSPoser::canGetCovariance(const sensor_msgs::NavSatFix & nav_sat_fix_msg)
{
  return nav_sat_fix_msg.position_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

GNSSStat GNSSPoser::convert(
  const sensor_msgs::NavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system)
{
  GNSSStat gnss_stat;
  if (coordinate_system == CoordinateSystem::UTM) {
    gnss_stat = NavSatFix2UTM(nav_sat_fix_msg);
  } else if (coordinate_system == CoordinateSystem::MGRS) {
    gnss_stat = NavSatFix2MGRS(nav_sat_fix_msg, MGRSPrecision::_100MICRO_METER);
  } else if (coordinate_system == CoordinateSystem::PLANE) {
    gnss_stat = NavSatFix2PLANE(nav_sat_fix_msg, plane_zone_);
  } else {
    ROS_ERROR_STREAM_THROTTLE(1, "Unknown Coordinate System");
  }
  return gnss_stat;
}

geometry_msgs::Point GNSSPoser::getPosition(const GNSSStat & gnss_stat)
{
  geometry_msgs::Point point;
  point.x = gnss_stat.x;
  point.y = gnss_stat.y;
  point.z = gnss_stat.z;
  return point;
}

geometry_msgs::Point GNSSPoser::getMedianPosition(
  const boost::circular_buffer<geometry_msgs::Point> & position_buffer)
{
  auto getMedian = [](std::vector<double> array) {
    std::sort(std::begin(array), std::end(array));
    const size_t median_index = array.size() / 2;
    double median = (array.size() % 2)
                      ? (array.at(median_index))
                      : ((array.at(median_index) + array.at(median_index - 1)) / 2);
    return median;
  };

  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::Point median_point;
  median_point.x = getMedian(array_x);
  median_point.y = getMedian(array_y);
  median_point.z = getMedian(array_z);
  return median_point;
}

geometry_msgs::Quaternion GNSSPoser::getQuaternionByHeading(const int heading)
{
  int heading_conv = 0;
  // convert heading[0(North)~360] to yaw[-M_PI(West)~M_PI]
  if (heading >= 0 && heading <= 27000000) {
    heading_conv = 9000000 - heading;
  } else {
    heading_conv = 45000000 - heading;
  }
  const double yaw = (heading_conv * 1e-5) * M_PI / 180.0;

  tf2::Quaternion quta;
  quta.setRPY(0, 0, yaw);

  return tf2::toMsg(quta);
}

geometry_msgs::Quaternion GNSSPoser::getQuaternionByPositionDiffence(
  const geometry_msgs::Point & point, const geometry_msgs::Point & prev_point)
{
  const double yaw = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
  tf2::Quaternion quta;
  quta.setRPY(0, 0, yaw);
  return tf2::toMsg(quta);
}

bool GNSSPoser::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN_STREAM_THROTTLE(1, ex.what());
    ROS_WARN_STREAM_THROTTLE(
      1, "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool GNSSPoser::getStaticTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
  const ros::Time & stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, stamp);
  } catch (tf2::TransformException & ex) {
    ROS_WARN_STREAM_THROTTLE(1, ex.what());
    ROS_WARN_STREAM_THROTTLE(
      1, "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());

    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void GNSSPoser::publishTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

}  // namespace GNSSPoser
