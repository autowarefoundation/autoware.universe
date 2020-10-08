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
#pragma once

#include <string>

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <ublox_msgs/NavPVT.h>

#include "gnss_poser/gnss_stat.h"

namespace GNSSPoser
{
class GNSSPoser
{
public:
  GNSSPoser(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  void callbackNavSatFix(const sensor_msgs::NavSatFix::ConstPtr & nav_sat_fix_msg_ptr);
  void callbackNavPVT(const ublox_msgs::NavPVT::ConstPtr & msg);

  bool isFixed(const sensor_msgs::NavSatStatus & nav_sat_status_msg);
  bool canGetCovariance(const sensor_msgs::NavSatFix & nav_sat_fix_msg);
  GNSSStat convert(
    const sensor_msgs::NavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system);
  geometry_msgs::Point getPosition(const GNSSStat & gnss_stat);
  geometry_msgs::Point getMedianPosition(
    const boost::circular_buffer<geometry_msgs::Point> & position_buffer);
  geometry_msgs::Quaternion getQuaternionByHeading(const int heading);
  geometry_msgs::Quaternion getQuaternionByPositionDiffence(
    const geometry_msgs::Point & point, const geometry_msgs::Point & prev_point);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);
  bool getStaticTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
    const ros::Time & stamp);
  void publishTF(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::PoseStamped & pose_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  ros::Subscriber nav_sta_fix_sub_;
  ros::Subscriber nav_pvt_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher pose_cov_pub_;
  ros::Publisher fixed_pub_;

  CoordinateSystem coordinate_system_;
  std::string base_frame_;
  std::string gnss_frame_;
  std::string gnss_base_frame_;
  std::string map_frame_;

  bool use_ublox_receiver_;

  int plane_zone_;

  boost::circular_buffer<geometry_msgs::Point> position_buffer_;
  ublox_msgs::NavPVT::ConstPtr nav_pvt_msg_ptr_;
};

}  // namespace GNSSPoser
