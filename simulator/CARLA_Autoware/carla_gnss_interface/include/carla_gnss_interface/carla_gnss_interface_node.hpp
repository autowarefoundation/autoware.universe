// Copyright 2024 Autoware Foundation. All rights reserved.
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

#ifndef CARLA_GNSS_INTERFACE__CARLA_GNSS_INTERFACE_NODE_HPP_
#define CARLA_GNSS_INTERFACE__CARLA_GNSS_INTERFACE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <math.h>
#include <proj.h>

#include <algorithm>
#include <string>
#include <vector>

class GnssInterface : public rclcpp::Node
{
public:
  explicit GnssInterface(const rclcpp::NodeOptions & node_options);
  std::string tf_output_frame_;

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_fix;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pup_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pup_pose_with_cov;

  void GnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

namespace interface
{
namespace gnss
{

class GPSPoint
{
public:
  double x, y, z;
  double lat, lon, alt;
  double dir, a;

  GPSPoint()
  {
    x = y = z = 0;
    lat = lon = alt = 0;
    dir = a = 0;
  }

  GPSPoint(const double & x, const double & y, const double & z, const double & a)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->a = a;

    lat = 0;
    lon = 0;
    alt = 0;
    dir = 0;
  }
};

class WayPoint
{
public:
  GPSPoint pos;
  WayPoint() {}

  WayPoint(const double & x, const double & y, const double & z, const double & a)
  {
    pos.x = x;
    pos.y = y;
    pos.z = z;
    pos.a = a;
  }
};

struct MappingParameters
{
  std::string proj_str;
  WayPoint org;
  double lat;
  double lon;
  double alt;
};

void llaToxyz(const MappingParameters & params, double & x_out, double & y_out, double & z_out);

}  // namespace gnss
}  // namespace interface

#endif  // CARLA_GNSS_INTERFACE__CARLA_GNSS_INTERFACE_NODE_HPP_
