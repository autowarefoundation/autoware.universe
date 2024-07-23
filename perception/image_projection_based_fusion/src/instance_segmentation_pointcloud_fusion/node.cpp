// Copyright 2024 TIER IV, Inc.
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

#include "autoware/image_projection_based_fusion/instance_segmentation_pointcloud_fusion/node.hpp"

#include "autoware/image_projection_based_fusion/utils/geometry.hpp"
#include "autoware/image_projection_based_fusion/utils/utils.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace autoware::image_projection_based_fusion
{
InstanceSegmentationPointCloudFusionNode::InstanceSegmentationPointCloudFusionNode(
  const rclcpp::NodeOptions & options)
: FusionNode<PointCloud2, PointCloud2, DetectedObjectsWithMask>(
    "instance_segmentation_pointcloud_fusion", options),
  transform_provider_ptr_(std::make_shared<TransformProvider>(this->get_clock()))
{
  filter_distance_threshold_ = declare_parameter<float>("filter_distance_threshold");
  for (auto & item : keep_instance_label_list_) {
    item.second = declare_parameter<bool>("keep_instance_label." + item.first);
  }
  for (const auto & item : keep_instance_label_list_) {
    RCLCPP_INFO(
      this->get_logger(), "filter_semantic_label_target: %s %d", item.first.c_str(), item.second);
  }

  pub_debug_image_ptr_ = this->create_publisher<Image>("~/debug_image", 1);
}

void InstanceSegmentationPointCloudFusionNode::preprocess(__attribute__((unused))
                                                          PointCloud2 & pointcloud_msg)
{
  return;
}

void InstanceSegmentationPointCloudFusionNode::postprocess(__attribute__((unused))
                                                           PointCloud2 & pointcloud_msg)
{
  return;
}

void InstanceSegmentationPointCloudFusionNode::fuseOnSingleImage(
  [[maybe_unused]] const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  [[maybe_unused]] const std::size_t image_id,
  [[maybe_unused]] const DetectedObjectsWithMask & input_objects,
  [[maybe_unused]] const sensor_msgs::msg::CameraInfo & camera_info,
  [[maybe_unused]] sensor_msgs::msg::PointCloud2 & output_pointcloud_msg)
{
  if (input_pointcloud_msg.data.empty()) {
    return;
  }
  if (input_objects.feature_objects.empty()) {
    return;
  }

  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info);

  if (!transform_stamped_map_[static_cast<int>(image_id)]) {
    std::optional<geometry_msgs::msg::TransformStamped> transform =
      (*transform_provider_ptr_)(camera_info.header.frame_id, input_pointcloud_msg.header.frame_id);
    if (!transform.has_value()) {
      return;
    }
    transform_stamped_map_[static_cast<int>(image_id)] = transform;
  }

  cv::Mat combined_mask = cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC1);
  std::map<int, cv::Mat> mask_map;
  for (size_t index = 0; index < input_objects.feature_objects.size(); index++) {
    const auto & object = input_objects.feature_objects[index];
    cv_bridge::CvImagePtr in_image_ptr;
    try {
      in_image_ptr = cv_bridge::toCvCopy(
        std::make_shared<sensor_msgs::msg::Image>(object.mask), object.mask.encoding);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception:%s", e.what());
      return;
    }

    cv::Mat resized_mask;
    cv::resize(
      in_image_ptr->image, resized_mask, cv::Size(camera_info.width, camera_info.height), 0, 0,
      cv::INTER_NEAREST);
    cv::bitwise_or(combined_mask, resized_mask, combined_mask);
    mask_map[static_cast<int>(index)] = resized_mask;
  }

  cv::cvtColor(combined_mask, combined_mask, cv::COLOR_GRAY2BGR);

  PointCloud2 transformed_cloud;
  tf2::doTransform(
    input_pointcloud_msg, transformed_cloud,
    transform_stamped_map_[static_cast<int>(image_id)].value());

  int point_step = input_pointcloud_msg.point_step;
  int x_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "x")].offset;
  int y_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "y")].offset;
  int z_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "z")].offset;
  size_t output_pointcloud_size = 0;
  output_pointcloud_msg.data.clear();
  output_pointcloud_msg.data.resize(input_pointcloud_msg.data.size());
  output_pointcloud_msg.fields = input_pointcloud_msg.fields;
  output_pointcloud_msg.header = input_pointcloud_msg.header;
  output_pointcloud_msg.height = input_pointcloud_msg.height;
  output_pointcloud_msg.point_step = input_pointcloud_msg.point_step;
  output_pointcloud_msg.is_bigendian = input_pointcloud_msg.is_bigendian;
  output_pointcloud_msg.is_dense = input_pointcloud_msg.is_dense;
  for (size_t global_offset = 0; global_offset < transformed_cloud.data.size();
       global_offset += point_step) {
    float transformed_x =
      *reinterpret_cast<float *>(&transformed_cloud.data[global_offset + x_offset]);
    float transformed_y =
      *reinterpret_cast<float *>(&transformed_cloud.data[global_offset + y_offset]);
    float transformed_z =
      *reinterpret_cast<float *>(&transformed_cloud.data[global_offset + z_offset]);
    // skip filtering pointcloud behind the camera or too far from camera
    if (transformed_z <= 0.0 || transformed_z > filter_distance_threshold_) {
      copyPointCloud(
        input_pointcloud_msg, point_step, global_offset, output_pointcloud_msg,
        output_pointcloud_size);
      continue;
    }

    Eigen::Vector2d projected_point = calcRawImageProjectedPoint(
      pinhole_camera_model, cv::Point3d(transformed_x, transformed_y, transformed_z));

    bool is_inside_image = projected_point.x() > 0 && projected_point.x() < camera_info.width &&
                           projected_point.y() > 0 && projected_point.y() < camera_info.height;
    if (!is_inside_image) {
      copyPointCloud(
        input_pointcloud_msg, point_step, global_offset, output_pointcloud_msg, output_pointcloud_size);
      continue;
    }

    for (const auto & item : mask_map) {
      const tier4_perception_msgs::msg::DetectedObjectWithMask & object =
        input_objects.feature_objects[item.first];
      uint8_t pixel_value = item.second.at<uint8_t>(
        static_cast<uint16_t>(projected_point.y()), static_cast<uint16_t>(projected_point.x()));
      if (
        keep_instance_label_list_[object.object.classification[0].label].second &&
        pixel_value > 199) {
        cv::circle(
          combined_mask,
          cv::Point(static_cast<int>(projected_point.x()), static_cast<int>(projected_point.y())),
          1, cv::Scalar(255, 0, 0), -1);
        copyPointCloud(
          input_pointcloud_msg, point_step, global_offset, output_pointcloud_msg,
          output_pointcloud_size);
      }
    }
  }

  Image debug_image;
  debug_image.header = camera_info.header;
  debug_image.encoding = "bgr8";
  debug_image.height = combined_mask.rows;
  debug_image.width = combined_mask.cols;
  debug_image.step = combined_mask.cols * combined_mask.elemSize();
  debug_image.data = std::vector<uint8_t>(
    combined_mask.data, combined_mask.data + combined_mask.total() * combined_mask.elemSize());
  pub_debug_image_ptr_->publish(debug_image);
}

bool InstanceSegmentationPointCloudFusionNode::out_of_scope(__attribute__((unused))
                                                            const PointCloud2 & filtered_cloud)
{
  return false;
}
}  // namespace autoware::image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::image_projection_based_fusion::InstanceSegmentationPointCloudFusionNode)