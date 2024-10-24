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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__INSTANCE_SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__INSTANCE_SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_

#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <autoware/image_projection_based_fusion/utils/utils.hpp>

#include <string>
#include <utility>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else

#include <cv_bridge/cv_bridge.h>

#endif

class TransformProvider
{
public:
  explicit TransformProvider(rclcpp::Clock::SharedPtr clock)
  {
    std::cout << "TransformProvider constructor" << std::endl;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  std::optional<geometry_msgs::msg::TransformStamped> operator()(
    const std::string & target_frame, const std::string & source_frame) const
  {
    std::optional<geometry_msgs::msg::TransformStamped> transform_stamped;
    try {
      transform_stamped =
        tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", ex.what());
      return std::nullopt;
    }
    return transform_stamped;
  }

private:
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

using TransformProviderPtr = std::shared_ptr<TransformProvider>;
using TransformProviderConstPtr = std::shared_ptr<const TransformProvider>;

namespace autoware::image_projection_based_fusion
{
class InstanceSegmentationPointCloudFusionNode : public FusionNode<PointCloud2, PointCloud2, Image>
{
public:
  explicit InstanceSegmentationPointCloudFusionNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_ptr_;
  rclcpp::Publisher<Image>::SharedPtr pub_debug_image_ptr_;
  float filter_distance_threshold_;
  TransformProviderConstPtr transform_provider_ptr_;
  std::map<int, std::optional<geometry_msgs::msg::TransformStamped>> transform_stamped_map_;
  std::vector<std::pair<std::string, bool>> keep_instance_label_list_ = {
    {"UNKNOWN", true}, {"CAR", true},        {"TRUCK", true},   {"BUS", true},
    {"TRAILER", true}, {"MOTORCYCLE", true}, {"BICYCLE", true}, {"PEDESTRIAN", true}};

protected:
  void preprocess(PointCloud2 & pointcloud_msg) override;

  void postprocess(PointCloud2 & pointcloud_msg) override;

  void fuseOnSingleImage(
    const PointCloud2 & input_pointcloud_msg, const std::size_t image_id, const Image & input_mask,
    const CameraInfo & camera_info, PointCloud2 & output_pointcloud_msg) override;

  bool out_of_scope(const PointCloud2 & filtered_cloud);

  inline void copyPointCloud(
    const PointCloud2 & input, const int point_step, const size_t global_offset,
    PointCloud2 & output, size_t & output_pointcloud_size)
  {
    std::memcpy(&output.data[output_pointcloud_size], &input.data[global_offset], point_step);
    output_pointcloud_size += point_step;
  }
};
}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__INSTANCE_SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_
