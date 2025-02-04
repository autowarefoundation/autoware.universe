// Copyright 2020-2023 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__DETECTOR_HPP_
#define AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__DETECTOR_HPP_

#include "autoware/lidar_apollo_instance_segmentation/cluster2d.hpp"
#include "autoware/lidar_apollo_instance_segmentation/feature_generator.hpp"
#include "autoware/lidar_apollo_instance_segmentation/node.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/stream_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/universe_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware
{
namespace lidar_apollo_instance_segmentation
{
using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

class LidarApolloInstanceSegmentation : public LidarInstanceSegmentationInterface
{
public:
  explicit LidarApolloInstanceSegmentation(rclcpp::Node * node);
  ~LidarApolloInstanceSegmentation() {}
  bool detectDynamicObjects(
    const sensor_msgs::msg::PointCloud2 & input,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & output) override;

private:
  bool transformCloud(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & transformed_cloud,
    float z_offset);

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_;

  rclcpp::Node * node_;
  std::shared_ptr<Cluster2D> cluster2d_;
  std::shared_ptr<FeatureGenerator> feature_generator_;
  float score_threshold_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_;
  float z_offset_;

  size_t output_size_;
  CudaUniquePtr<float[]> input_d_;
  CudaUniquePtr<float[]> output_d_;
  CudaUniquePtrHost<float[]> output_h_;

  StreamUniquePtr stream_{makeCudaStream()};
};
}  // namespace lidar_apollo_instance_segmentation
}  // namespace autoware

#endif  // AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__DETECTOR_HPP_
