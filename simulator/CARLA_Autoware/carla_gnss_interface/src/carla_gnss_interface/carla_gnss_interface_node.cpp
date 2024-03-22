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

#include "carla_gnss_interface/carla_gnss_interface_node.hpp"

#include <proj.h>

namespace interface
{
namespace gnss
{

MappingUtils::MappingUtils()
{
}

void MappingUtils::llaToxyz(const MappingParameters &params, double &x_out, double &y_out, double &z_out) {
    if (params.proj_str.size() < 8) return;

  PJ_CONTEXT * C = proj_context_create();
  PJ * P = proj_create_crs_to_crs(C, "EPSG:4326", params.proj_str.c_str(), NULL);

  if (P == nullptr) {
    proj_context_destroy(C);
    return;
  }

  PJ_COORD gps_degrees = proj_coord(params.lat, params.lon, params.alt, 0);
  PJ_COORD xyz_out = proj_trans(P, PJ_FWD, gps_degrees);
  x_out = xyz_out.enu.e + params.org.pos.x;
  y_out = xyz_out.enu.n + params.org.pos.y;
  z_out = xyz_out.enu.u + params.org.pos.z;

  proj_destroy(P);
  proj_context_destroy(C);
}
}  // namespace gnss
}  // namespace interface

void GnssInterface::GnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_;
  interface::gnss::WayPoint origin, p;
  // Create an instance of MappingUtils
  interface::gnss::MappingUtils mappingUtils;

  // Create MappingParameters struct to hold the parameters
  interface::gnss::MappingParameters params;
  params.proj_str =
    "+proj=tmerc +lat_0=0 +lon_0=0 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs";
  params.lat = msg->latitude;
  params.lon = msg->longitude;
  params.alt = msg->altitude;

  // Call llaToxyz function
  mappingUtils.llaToxyz(params, p.pos.x, p.pos.y, p.pos.z);

  pose_.header = msg->header;
  pose_.header.frame_id = "map";
  pose_.pose.position.x = p.pos.x;
  pose_.pose.position.y = p.pos.y;
  pose_.pose.position.z = p.pos.z;

  pose_cov_.header = pose_.header;
  pose_cov_.pose.pose = pose_.pose;

  pup_pose->publish(pose_);
  pup_pose_with_cov->publish(pose_cov_);
}


GnssInterface::GnssInterface(const rclcpp::NodeOptions & node_options)
: Node("gnss_interface_node", node_options), tf_output_frame_("base_link")
{
  sub_gnss_fix = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "carla/ego_vehicle/gnss", 1,
    std::bind(&GnssInterface::GnssCallBack, this, std::placeholders::_1));

  pup_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sensing/gnss/pose", 1);
  pup_pose_with_cov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/sensing/gnss/pose_with_covariance", 1);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GnssInterface)
