// Copyright 2022 Autoware Foundation
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

#include "map_fit_module.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

MapFitModule::MapFitModule(rclcpp::Node * node)
: logger_(node->get_logger()), tf2_listener_(tf2_buffer_)
{
  sub_map_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapFitModule::OnMap, this, std::placeholders::_1));
}

void MapFitModule::OnMap(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  map_frame_ = msg->header.frame_id;
  map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *map_cloud_);
}

double MapFitModule::GetGroundHeight(const tf2::Vector3 & point) const
{
  constexpr double radius = 1.0 * 1.0;
  const double x = point.getX();
  const double y = point.getY();

  double height = INFINITY;
  for (const auto & p : map_cloud_->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }
  RCLCPP_WARN_STREAM(logger_, "ground height: " << height);
  return std::isfinite(height) ? height : point.getZ();
}

PoseWithCovarianceStamped MapFitModule::FitHeight(const PoseWithCovarianceStamped pose) const
{
  const auto & position = pose.pose.pose.position;
  tf2::Vector3 point(position.x, position.y, position.z);
  std::string fixed_frame = pose.header.frame_id;

  if (map_cloud_) {
    try {
      const auto stamped = tf2_buffer_.lookupTransform(map_frame_, fixed_frame, tf2::TimePointZero);
      tf2::Transform transform{tf2::Quaternion{}, tf2::Vector3{}};
      tf2::fromMsg(stamped.transform, transform);
      point = transform * point;
      point.setZ(GetGroundHeight(point));
      point = transform.inverse() * point;
    } catch (tf2::TransformException & exception) {
      RCLCPP_WARN_STREAM(logger_, "failed to lookup transform: " << exception.what());
    }
  }

  PoseWithCovarianceStamped result = pose;
  result.pose.pose.position.x = point.getX();
  result.pose.pose.position.y = point.getY();
  result.pose.pose.position.z = point.getZ();
  return result;
}
