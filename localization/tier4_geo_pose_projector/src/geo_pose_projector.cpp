// Copyright 2023 Autoware Foundation
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

#include "geo_pose_projector.hpp"

#include <geography_utils/height.hpp>
#include <geography_utils/projection.hpp>

GeoPoseProjector::GeoPoseProjector() : Node("geo_pose_projector")
{
  // Subscribe to map_projector_info topic
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(
    sub_map_projector_info_,
    [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { projector_info_ = *msg; });

  // Subscribe to geo_pose topic
  geo_pose_sub_ = create_subscription<GeoPoseWithCovariance>(
    "input_geo_pose", 10, [this](const GeoPoseWithCovariance::SharedPtr msg) { on_geo_pose(msg); });

  // Publish pose topic
  pose_pub_ = create_publisher<PoseWithCovariance>("output_pose", 10);
}

void GeoPoseProjector::on_geo_pose(const GeoPoseWithCovariance::SharedPtr msg)
{
  if (!projector_info_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000 /* ms */, "map_projector_info is not received yet.");
    return;
  }

  // get position
  geographic_msgs::msg::GeoPoint gps_point;
  gps_point.latitude = msg->pose.pose.position.latitude;
  gps_point.longitude = msg->pose.pose.position.longitude;
  gps_point.altitude = msg->pose.pose.position.altitude;
  geometry_msgs::msg::Point position =
    geography_utils::project_forward(gps_point, projector_info_.value());
  position.z = geography_utils::convert_height(
    position.z, gps_point.latitude, gps_point.longitude, MapProjectorInfo::Message::WGS84,
    projector_info_.value().vertical_datum);

  // Convert geo_pose to pose
  PoseWithCovariance projected_pose;
  projected_pose.header = msg->header;
  projected_pose.pose.pose.position = position;
  pose_pub_->publish(projected_pose);
}
