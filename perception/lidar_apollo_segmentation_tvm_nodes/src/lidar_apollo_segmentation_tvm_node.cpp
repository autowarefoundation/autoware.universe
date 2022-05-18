// Copyright 2020-2022 Arm Ltd., TierIV
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

#include <common/types.hpp>
#include <lidar_apollo_segmentation_tvm/lidar_apollo_segmentation_tvm.hpp>
#include <lidar_apollo_segmentation_tvm_nodes/lidar_apollo_segmentation_tvm_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::perception::lidar_apollo_segmentation_tvm::ApolloLidarSegmentation;

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm_nodes
{

ApolloLidarSegmentationNode::ApolloLidarSegmentationNode(const rclcpp::NodeOptions & options)
: Node("lidar_apollo_segmentation_tvm", options),
  m_cloud_sub_ptr{create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_in", rclcpp::QoS{10},
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pointCloudCallback(msg); })},
  m_box_pub_ptr{create_publisher<autoware_auto_perception_msgs::msg::BoundingBoxArray>(
    "lidar_bounding_boxes", rclcpp::QoS{10})},
  m_marker_pub_ptr{create_publisher<visualization_msgs::msg::MarkerArray>(
    "lidar_bounding_boxes_viz", rclcpp::QoS{10})},
  m_detector_ptr{std::make_shared<lidar_apollo_segmentation_tvm::ApolloLidarSegmentation>(
    declare_parameter("range", rclcpp::ParameterValue{70}).get<int32_t>(),
    declare_parameter("score_threshold", rclcpp::ParameterValue{0.8}).get<float32_t>(),
    declare_parameter("use_intensity_feature", rclcpp::ParameterValue{true}).get<bool8_t>(),
    declare_parameter("use_constant_feature", rclcpp::ParameterValue{false}).get<bool8_t>(),
    declare_parameter("z_offset", rclcpp::ParameterValue{0.0}).get<float32_t>(),
    declare_parameter("min_height", rclcpp::ParameterValue{-5.0}).get<float32_t>(),
    declare_parameter("max_height", rclcpp::ParameterValue{5.0}).get<float32_t>(),
    declare_parameter("objectness_thresh", rclcpp::ParameterValue{0.5}).get<float32_t>(),
    declare_parameter("min_pts_num", rclcpp::ParameterValue{3}).get<int32_t>(),
    declare_parameter("height_thresh", rclcpp::ParameterValue{0.5}).get<float32_t>())}
{
}

void ApolloLidarSegmentationNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  std::shared_ptr<const autoware_auto_perception_msgs::msg::BoundingBoxArray> output_msg;
  try {
    output_msg = m_detector_ptr->detectDynamicObjects(*msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), e.what());
    return;
  }
  m_box_pub_ptr->publish(*output_msg);

  // Also publish boxes for visualization
  uint32_t id_counter = 0;
  visualization_msgs::msg::MarkerArray marker_array;
  for (const auto & box : output_msg->boxes) {
    visualization_msgs::msg::Marker m{};
    m.header.stamp = rclcpp::Time(0);
    m.header.frame_id = output_msg->header.frame_id;
    m.ns = "bbox";
    m.id = static_cast<int>(id_counter);
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = static_cast<float64_t>(box.centroid.x);
    m.pose.position.y = static_cast<float64_t>(box.centroid.y);
    m.pose.position.z = static_cast<float64_t>(box.centroid.z);
    m.pose.orientation.x = static_cast<float64_t>(box.orientation.x);
    m.pose.orientation.y = static_cast<float64_t>(box.orientation.y);
    m.pose.orientation.z = static_cast<float64_t>(box.orientation.z);
    m.pose.orientation.w = static_cast<float64_t>(box.orientation.w);
    // X and Y scale are swapped between these two message types
    m.scale.x = static_cast<float64_t>(box.size.y);
    m.scale.y = static_cast<float64_t>(box.size.x);
    m.scale.z = static_cast<float64_t>(box.size.z);
    m.color.r = 1.0;
    m.color.g = 0.5;
    m.color.b = 0.0;
    m.color.a = 0.75;
    m.lifetime.sec = 0;
    m.lifetime.nanosec = 500000000;
    marker_array.markers.push_back(m);
    id_counter++;
  }
  m_marker_pub_ptr->publish(marker_array);
}
}  // namespace lidar_apollo_segmentation_tvm_nodes
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::lidar_apollo_segmentation_tvm_nodes::ApolloLidarSegmentationNode)
