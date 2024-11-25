// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <thrust/device_vector.h>

#include <cmath>
#include <deque>

namespace autoware::cuda_pointcloud_preprocessor
{

struct TwistStruct2D
{
  float cum_x;
  float cum_y;
  float cum_theta;
  float cum_cos_theta;
  float cum_sin_theta;
  uint32_t last_stamp_nsec;  // relative to the start of the pointcloud
  uint32_t stamp_nsec;       // relative to the start of the pointcloud
  float vx;
  float vtheta;
};

struct TwistStruct3D
{
  // Eigen::Affine3f cum_transform;
  float cum_transform_buffer[16];
  /* float cum_x;
  float cum_y;
  float cum_theta; */
  /* float cum_cos_theta;
  float cum_sin_theta; */
  uint32_t last_stamp_nsec;  // relative to the start of the pointcloud
  uint32_t stamp_nsec;       // relative to the start of the pointcloud
  /* float vx;
  float vtheta; */
  float v[3];
  float w[3];
};

struct TransformStruct
{
  float x;
  float y;
  float z;
  float m11;
  float m12;
  float m13;
  float m21;
  float m22;
  float m23;
  float m31;
  float m32;
  float m33;
};

struct CropBoxParameters
{
  float min_x{std::nanf("")};
  float max_x{std::nanf("")};
  float min_y{std::nanf("")};
  float max_y{std::nanf("")};
  float min_z{std::nanf("")};
  float max_z{std::nanf("")};
};

struct RingOutlierFilterParameters
{
  float distance_ratio{std::nanf("")};
  float object_length_threshold{std::nanf("")};
  std::size_t num_points_threshold{0};
};

class CudaPointcloudPreprocessor
{
public:
  CudaPointcloudPreprocessor();
  ~CudaPointcloudPreprocessor() = default;

  void setCropBoxParameters(
    const CropBoxParameters & self_crop_box_parameters,
    const CropBoxParameters & mirror_crop_box_parameters);
  void setRingOutlierFilterParameters(const RingOutlierFilterParameters & ring_outlier_parameters);
  void set3DUndistortion(bool value);

  void preallocateOutput();

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> process(
    const cuda_blackboard::CudaPointCloud2 & input_pointcloud_msg,
    const geometry_msgs::msg::TransformStamped & transform_msg,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
    const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue);

private:
  void setupTwist2DStructs(
    const cuda_blackboard::CudaPointCloud2 & input_pointcloud_msg,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
    const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue);

  void setupTwist3DStructs(
    const cuda_blackboard::CudaPointCloud2 & input_pointcloud_msg,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
    const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue);

  CropBoxParameters self_crop_box_parameters_{};
  CropBoxParameters mirror_crop_box_parameters_{};
  RingOutlierFilterParameters ring_outlier_parameters_{};
  bool use_3d_undistortion_{false};

  int max_rings_{};
  int max_points_per_ring_{};

  std::vector<sensor_msgs::msg::PointField> point_fields_{};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> output_pointcloud_ptr_{};

  thrust::device_vector<InputPointType> device_transformed_points_{};
  thrust::device_vector<OutputPointType> device_output_points_{};
  thrust::device_vector<uint32_t> device_self_crop_mask_{};
  thrust::device_vector<uint32_t> device_mirror_crop_mask_{};
  thrust::device_vector<uint32_t> device_ring_outlier_mask_{};
  thrust::device_vector<uint32_t> device_indices_{};
  thrust::device_vector<TwistStruct2D> device_twist_2d_structs_{};
  thrust::device_vector<TwistStruct3D> device_twist_3d_structs_{};
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_HPP_
