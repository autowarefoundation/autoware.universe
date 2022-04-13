// Copyright 2022 TIER IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING__NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING__NODE_HPP_

#include "image_projection_based_fusion/fusion_node.hpp"

#include <centerpoint_trt.hpp>
#include <config.hpp>
#include <image_projection_based_fusion/utils/geometry.hpp>
#include <image_projection_based_fusion/utils/utils.hpp>

#include <map>
#include <memory>
#include <vector>

namespace image_projection_based_fusion
{
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
class PointpaintingFusionNode : public FusionNode<sensor_msgs::msg::PointCloud2>
{
public:
  explicit PointpaintingFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg) override;

  void fuseOnSingleImage(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const int image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info,
    sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg) override;

  void paintPoints(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const DetectedObjectsWithFeature & input_roi_msg, const double image_width,
    const double image_height, const geometry_msgs::msg::TransformStamped transform_stamped,
    const Eigen::Matrix4d & camera_projection,
    sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg);

  void postprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg) override;

  void box3DToDetectedObject(
    const Box3D & box3d, autoware_auto_perception_msgs::msg::DetectedObject & obj);
  static uint8_t getSemanticType(const std::string & class_name);
  static bool isCarLikeVehicleLabel(const uint8_t label);

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr obj_pub_ptr_;

  float score_threshold_{0.0};
  std::vector<std::string> class_names_;
  bool rename_car_to_truck_and_bus_{false};

  std::unique_ptr<CenterPointTRT> detector_ptr_{nullptr};
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING__NODE_HPP_
