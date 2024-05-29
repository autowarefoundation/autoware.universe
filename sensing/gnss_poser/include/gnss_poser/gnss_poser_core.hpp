// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef GNSS_POSER__GNSS_POSER_CORE_HPP_
#define GNSS_POSER__GNSS_POSER_CORE_HPP_

#include "gnss_poser/convert.hpp"
#include "gnss_poser/gnss_stat.hpp"

// #include <rclcpp/rclcpp.hpp>

// #include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <boost/circular_buffer.hpp>
// #include <tf2/transform_datatypes.h>
// #ifdef ROS_DISTRO_GALACTIC
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #endif
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

#include <string>
#include "gnss_poser/msg.hpp"
namespace gnss_poser
{
class GNSSPoser 
{
public:
  explicit GNSSPoser(void *dora_context,CoordinateSystem coordinate_system);
  void callbackNavSatFix(const DoraNavSatFix nav_sat_fix_msg_ptr);
  // GNSSPoser::GNSSPoser(void *dora_context);
private:
  // void callbackNavSatFix(const DoraNavSatFix nav_sat_fix_msg_ptr);
  // void callbackGnssInsOrientationStamped(
  //   const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg);

  bool isFixed(const NavSatStatus & nav_sat_status_msg);
  bool canGetCovariance(const DoraNavSatFix & nav_sat_fix_msg);
  GNSSStat convert(
  const DoraNavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system);
  Point getPosition(const GNSSStat & gnss_stat);
  Point getMedianPosition(const boost::circular_buffer<Point> & position_buffer);
      
  geometry_msgs::Quaternion getQuaternionByPositionDifference(const Point & point, Point & prev_point);
  // geometry_msgs::msg::Point getMedianPosition(
  //   const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);
  // geometry_msgs::msg::Quaternion getQuaternionByHeading(const int heading);
  // geometry_msgs::msg::Quaternion getQuaternionByPositionDifference(
  //   const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point);

  // bool getTransform(
  //   const std::string & target_frame, const std::string & source_frame,
  //   const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr);
  // bool getStaticTransform(
  //   const std::string & target_frame, const std::string & source_frame,
  //   const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr,
  //   const builtin_interfaces::msg::Time & stamp);
  // void publishTF(
  //   const std::string & frame_id, const std::string & child_frame_id,
  //   const geometry_msgs::msg::PoseStamped & pose_msg);

  CoordinateSystem coordinate_system_;
  std::string base_frame_;
  std::string gnss_frame_;
  std::string gnss_base_frame_;
  std::string map_frame_;

  bool use_gnss_ins_orientation_;

  boost::circular_buffer<Point> position_buffer_;
  GnssInsOrientationStamped msg_gnss_ins_orientation_stamped_;
  int plane_zone_;
  void *GNSSPoser_dora_context;
};
}  // namespace gnss_poser

#endif  // GNSS_POSER__GNSS_POSER_CORE_HPP_
