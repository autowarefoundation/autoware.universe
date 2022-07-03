// Copyright 2020 TierIV
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

#ifndef DETECTED_OBJECT_FILTER__DETECTED_OBJECT_FILTER_HPP_
#define DETECTED_OBJECT_FILTER__DETECTED_OBJECT_FILTER_HPP_

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <boost/geometry/geometry.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <string>

using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Point3d;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

#include <memory>

namespace detected_object_filter
{
class DetectedObjectFilterNode : public rclcpp::Node
{
public:
  explicit DetectedObjectFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg);
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  float voxel_size_x_;
  float voxel_size_y_;
  bool filter_by_xy_value_;
  float upper_bound_x_;
  float upper_bound_y_;
  float lower_bound_x_;
  float lower_bound_y_;

  bool transformPointCloud(
    const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
    PointCloud2 * out_cloud_ptr);
};

}  // namespace detected_object_filter

#endif  // DETECTED_OBJECT_FILTER__DETECTED_OBJECT_FILTER_HPP_
