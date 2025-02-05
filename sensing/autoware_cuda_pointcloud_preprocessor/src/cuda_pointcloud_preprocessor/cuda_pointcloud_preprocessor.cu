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

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cub/cub.cuh>

#include <cuda_runtime.h>
#include <tf2/utils.h>
#include <thrust/execution_policy.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>

namespace autoware::cuda_pointcloud_preprocessor
{

__host__ __device__ Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f & v)
{
  Eigen::Matrix3f m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

__host__ __device__ Eigen::Matrix3f leftJacobianSO3(const Eigen::Vector3f & omega)
{
  double theta = omega.norm();
  if (std::abs(theta) < 1e-6) {
    return Eigen::Matrix3f::Identity();
  }

  Eigen::Matrix3f Omega = skewSymmetric(omega);

  Eigen::Matrix3f Omega2 = Omega * Omega;
  double theta2 = theta * theta;
  double theta3 = theta2 * theta;

  // Rodrigues' formula for Jacobian
  return Eigen::Matrix3f::Identity() + ((1 - cos(theta)) / theta2) * Omega +
         ((theta - sin(theta)) / theta3) * Omega2;
}

__host__ __device__ Eigen::Matrix4f transformationMatrixFromVelocity(
  const Eigen::Vector3f & linear_velocity, const Eigen::Vector3f & angular_velocity, double dt)
{
  Eigen::Matrix3f R = Eigen::AngleAxisf(angular_velocity.norm() * dt, angular_velocity.normalized())
                        .toRotationMatrix();
  Eigen::Matrix3f J = leftJacobianSO3(angular_velocity * dt);

  Eigen::Vector3f translation = J * (linear_velocity * dt);

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = R;
  transformation.block<3, 1>(0, 3) = translation;

  return transformation;
}

__global__ void organizeKernel(
  const InputPointType * __restrict__ input_points, std::uint32_t * index_tensor,
  std::int32_t * ring_indexes, std::int32_t initial_max_rings, std::int32_t * output_max_rings,
  std::int32_t initial_max_points_per_ring, std::int32_t * output_max_points_per_ring,
  int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }

  auto ring = input_points[idx].channel;

  if (ring >= initial_max_rings) {
    atomicMax(output_max_rings, ring);
    return;
  }

  int next_offset = atomicAdd(&ring_indexes[ring], 1);

  if (next_offset >= initial_max_points_per_ring) {
    atomicMax(output_max_points_per_ring, next_offset);
    return;
  }

  index_tensor[ring * initial_max_points_per_ring + next_offset] = idx;
}

__global__ void gatherKernel(
  const InputPointType * __restrict__ input_points, const std::uint32_t * __restrict__ index_tensor,
  InputPointType * __restrict__ output_points, int num_rings, int max_points_per_ring)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_rings * max_points_per_ring) {
    return;
  }

  const int ring = idx / max_points_per_ring;
  const int point = idx % max_points_per_ring;

  const std::uint32_t input_idx = index_tensor[ring * max_points_per_ring + point];

  if (input_idx < std::numeric_limits<std::uint32_t>::max()) {
    output_points[ring * max_points_per_ring + point] = input_points[input_idx];
  } else {
    output_points[ring * max_points_per_ring + point].distance = 0.0f;
  }
}

__global__ void transformPointsKernel(
  const InputPointType * __restrict__ input_points, InputPointType * output_points, int num_points,
  TransformStruct transform)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    output_points[idx] = input_points[idx];

    const float x = input_points[idx].x;
    const float y = input_points[idx].y;
    const float z = input_points[idx].z;

    output_points[idx].x = transform.m11 * x + transform.m12 * y + transform.m13 * z + transform.x;
    output_points[idx].y = transform.m21 * x + transform.m22 * y + transform.m23 * z + transform.y;
    output_points[idx].z = transform.m31 * x + transform.m32 * y + transform.m33 * z + transform.z;
  }
}

__global__ void cropBoxKernel(
  InputPointType * __restrict__ d_points, std::uint32_t * __restrict__ output_mask, int num_points,
  const CropBoxParameters * __restrict__ crop_box_parameters_ptr, int num_crop_boxes)
{
  for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < num_points;
       idx += blockDim.x * gridDim.x) {
    const float x = d_points[idx].x;
    const float y = d_points[idx].y;
    const float z = d_points[idx].z;

    std::uint32_t mask = 1;

    for (int i = 0; i < num_crop_boxes; i++) {
      const CropBoxParameters & crop_box_parameters = crop_box_parameters_ptr[i];
      const float & min_x = crop_box_parameters.min_x;
      const float & min_y = crop_box_parameters.min_y;
      const float & min_z = crop_box_parameters.min_z;
      const float & max_x = crop_box_parameters.max_x;
      const float & max_y = crop_box_parameters.max_y;
      const float & max_z = crop_box_parameters.max_z;
      mask &=
        (x <= min_x || x >= max_x) || (y <= min_y || y >= max_y) || (z <= min_z || z >= max_z);
    }

    output_mask[idx] = mask;
  }
}

__global__ void combineMasksKernel(
  const std::uint32_t * __restrict__ mask1, const std::uint32_t * __restrict__ mask2,
  int num_points, std::uint32_t * __restrict__ output_mask)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    output_mask[idx] = mask1[idx] & mask2[idx];
  }
}

__global__ void extractInputPointIndicesKernel(
  InputPointType * input_points, std::uint32_t * masks, std::uint32_t * indices, int num_points,
  InputPointType * output_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    output_points[indices[idx] - 1] = input_points[idx];
  }
}

__global__ void extractOutputPointIndicesKernel(
  OutputPointType * input_points, std::uint32_t * masks, std::uint32_t * indices, int num_points,
  OutputPointType * output_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    output_points[indices[idx] - 1] = input_points[idx];
  }
}

__global__ void extractInputPointsToOutputPoints_indicesKernel(
  InputPointType * input_points, std::uint32_t * masks, std::uint32_t * indices, int num_points,
  OutputPointType * output_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    InputPointType & input_point = input_points[idx];
    OutputPointType & output_point = output_points[indices[idx] - 1];
    output_point.x = input_point.x;
    output_point.y = input_point.y;
    output_point.z = input_point.z;
    output_point.intensity = input_point.intensity;
    output_point.return_type = input_point.return_type;
    output_point.channel = input_point.channel;
  }
}

__global__ void undistort2dKernel(
  InputPointType * input_points, int num_points, TwistStruct2D * twist_structs, int num_twists)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    InputPointType & point = input_points[idx];

    // The twist must always be newer than the point ! (or it was the last twist)
    int twist_index = 0;
    while (twist_index < num_twists && twist_structs[twist_index].stamp_nsec < point.time_stamp) {
      twist_index++;
    }

    twist_index = min(twist_index, num_twists - 1);

    TwistStruct2D twist = twist_structs[twist_index];
    float x = twist.cum_x;
    float y = twist.cum_y;
    float theta = twist.cum_theta;

    double dt_nsec =
      point.time_stamp > twist.last_stamp_nsec ? point.time_stamp - twist.last_stamp_nsec : 0;
    double dt = 1e-9 * (dt_nsec);

    theta += twist.v_theta * dt;
    float d = twist.v_x * dt;
    x += d * cos(theta);
    y += d * sin(theta);

    float distorted_x = point.x;
    float distorted_y = point.y;

    point.x = distorted_x * cos(theta) - distorted_y * sin(theta) + x;
    point.y = distorted_x * sin(theta) + distorted_y * cos(theta) + y;
  }
}

__global__ void undistort3dKernel(
  InputPointType * input_points, int num_points, TwistStruct3D * twist_structs, int num_twists)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    InputPointType & point = input_points[idx];

    // The twist must always be newer than the point ! (or it was the last twist)
    int twist_index = 0;
    while (twist_index < num_twists && twist_structs[twist_index].stamp_nsec < point.time_stamp) {
      twist_index++;
    }

    twist_index = min(twist_index, num_twists - 1);

    TwistStruct3D twist = twist_structs[twist_index];
    Eigen::Map<Eigen::Matrix4f> cum_transform_buffer_map(twist.cum_transform_buffer);
    Eigen::Map<Eigen::Vector3f> v_map(twist.v);
    Eigen::Map<Eigen::Vector3f> w_map(twist.w);

    double dt_nsec =
      point.time_stamp > twist.last_stamp_nsec ? point.time_stamp - twist.last_stamp_nsec : 0;
    double dt = 1e-9 * (dt_nsec);

    Eigen::Matrix4f transform =
      cum_transform_buffer_map * transformationMatrixFromVelocity(v_map, w_map, dt);
    Eigen::Vector3f p(point.x, point.y, point.z);
    p = transform.block<3, 3>(0, 0) * p + transform.block<3, 1>(0, 3);

    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
  }
}

__global__ void ringOutlierFilterKernel(
  const InputPointType * d_points, std::uint32_t * output_mask, int num_rings,
  int max_points_per_ring, float distance_ratio, float object_length_threshold_squared,
  int num_points_threshold)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int j = idx / max_points_per_ring;
  int i = idx % max_points_per_ring;

  if (j >= num_rings || i >= max_points_per_ring) {
    return;
  }

  int min_i = max(i - num_points_threshold, 0);
  int max_i = min(i + num_points_threshold, max_points_per_ring);

  int walk_size = 1;
  int left_idx = min_i;
  int right_idx = min_i + 1;

  for (int k = min_i; k < max_i - 1; k++) {
    const InputPointType & left_point = d_points[j * max_points_per_ring + k];
    const InputPointType & right_point = d_points[j * max_points_per_ring + k + 1];

    // Find biggest walk that passes through i
    float azimuth_diff = right_point.azimuth - left_point.azimuth;
    azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 2 * M_PI : azimuth_diff;

    if (
      max(left_point.distance, right_point.distance) <
        min(left_point.distance, right_point.distance) * distance_ratio &&
      azimuth_diff < 1.f * M_PI / 180.f) {
      // Determined to be included in the same walk
      walk_size++;
      right_idx++;
    } else if (k >= i) {
      break;
    } else {
      walk_size = 1;
      left_idx = k + 1;
      right_idx = k + 2;  // this is safe since we break if k >= i
    }
  }

  const InputPointType & left_point = d_points[j * max_points_per_ring + left_idx];
  const InputPointType & right_point = d_points[j * max_points_per_ring + right_idx - 1];
  const float x = left_point.x - right_point.x;
  const float y = left_point.y - right_point.y;
  const float z = left_point.z - right_point.z;

  output_mask[j * max_points_per_ring + i] = static_cast<std::uint32_t>(
    (walk_size > num_points_threshold) ||
    (x * x + y * y + z * z >= object_length_threshold_squared));
}

__global__ void transformPointTypeKernel(
  const InputPointType * device_input_points, int num_points, OutputPointType * device_ouput_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    const InputPointType & input_point = device_input_points[idx];
    OutputPointType & output_point = device_ouput_points[idx];

    output_point.x = input_point.x;
    output_point.y = input_point.y;
    output_point.z = input_point.z;
    output_point.intensity = (float)input_point.intensity;
  }
}

CudaPointcloudPreprocessor::CudaPointcloudPreprocessor()
{
  sensor_msgs::msg::PointField x_field, y_field, z_field, intensity_field, return_type_field,
    channel_field, azimuth_field, elevation_field, distance_field, time_stamp_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;

  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;

  intensity_field.name = "intensity";
  intensity_field.offset = 12;
  intensity_field.datatype = sensor_msgs::msg::PointField::UINT8;
  intensity_field.count = 1;

  return_type_field.name = "return_type";
  return_type_field.offset = 13;
  return_type_field.datatype = sensor_msgs::msg::PointField::UINT8;
  return_type_field.count = 1;

  channel_field.name = "channel";
  channel_field.offset = 14;
  channel_field.datatype = sensor_msgs::msg::PointField::UINT16;
  channel_field.count = 1;

  static_assert(sizeof(OutputPointType) == 16, "OutputPointType size is not 16 bytes");
  static_assert(offsetof(OutputPointType, x) == 0);
  static_assert(offsetof(OutputPointType, y) == 4);
  static_assert(offsetof(OutputPointType, z) == 8);
  static_assert(offsetof(OutputPointType, intensity) == 12);
  static_assert(offsetof(OutputPointType, return_type) == 13);
  static_assert(offsetof(OutputPointType, channel) == 14);

  point_fields_.push_back(x_field);
  point_fields_.push_back(y_field);
  point_fields_.push_back(z_field);
  point_fields_.push_back(intensity_field);
  point_fields_.push_back(return_type_field);
  point_fields_.push_back(channel_field);

  cudaStreamCreate(&stream_);

  int num_sm;
  cudaDeviceGetAttribute(&num_sm, cudaDevAttrMultiProcessorCount, 0);
  max_blocks_per_grid_ = 4 * num_sm;  // used for strided loops

  cudaMemPoolProps pool_props;
  memset(&pool_props, 0, sizeof(cudaMemPoolProps));
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.handleTypes = cudaMemHandleTypePosixFileDescriptor;

  pool_props.location.type = cudaMemLocationTypeDevice;
  cudaGetDevice(&(pool_props.location.id));

  cudaMemPoolCreate(&device_memory_pool_, &pool_props);
  MemoryPoolAllocator<TwistStruct2D> allocator_2d(device_memory_pool_);
  MemoryPoolAllocator<TwistStruct3D> allocator_3d(device_memory_pool_);
  MemoryPoolAllocator<std::int32_t> allocator_int32(device_memory_pool_);
  MemoryPoolAllocator<std::uint32_t> allocator_uint32(device_memory_pool_);
  MemoryPoolAllocator<std::uint8_t> allocator_uint8(device_memory_pool_);
  MemoryPoolAllocator<InputPointType> allocator_points(device_memory_pool_);

  device_twist_2d_structs_ =
    thrust::device_vector<TwistStruct2D, MemoryPoolAllocator<TwistStruct2D>>(allocator_2d);
  device_twist_3d_structs_ =
    thrust::device_vector<TwistStruct3D, MemoryPoolAllocator<TwistStruct3D>>(allocator_3d);

  num_rings_ = 1;
  max_points_per_ring_ = 1;
  num_organized_points_ = num_rings_ * max_points_per_ring_;
  device_ring_index_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_ring_index_.resize(num_rings_);

  device_indexes_tensor_ =
    thrust::device_vector<std::uint32_t, MemoryPoolAllocator<std::uint32_t>>(allocator_uint32);
  device_sorted_indexes_tensor_ =
    thrust::device_vector<std::uint32_t, MemoryPoolAllocator<std::uint32_t>>(allocator_uint32);

  device_indexes_tensor_.resize(num_organized_points_);
  device_sorted_indexes_tensor_.resize(num_organized_points_);

  device_segment_offsets_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_segment_offsets_.resize(num_rings_ + 1);
  device_segment_offsets_[0] = 0;
  device_segment_offsets_[1] = 1;

  device_max_ring_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_max_ring_.resize(1);

  device_max_points_per_ring_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_max_points_per_ring_.resize(1);

  device_input_points_ =
    thrust::device_vector<InputPointType, MemoryPoolAllocator<InputPointType>>(allocator_points);
  device_organized_points_ =
    thrust::device_vector<InputPointType, MemoryPoolAllocator<InputPointType>>(allocator_points);
  device_organized_points_.resize(num_organized_points_);

  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_max_ring_.data()), 0, sizeof(std::int32_t), stream_);
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_max_points_per_ring_.data()), 0, sizeof(std::int32_t), stream_);
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_indexes_tensor_.data()), 0x255, sizeof(std::uint32_t), stream_);

  sort_workspace_bytes_ = querySortWorkspace(
    num_rings_ * max_points_per_ring_, num_rings_,
    thrust::raw_pointer_cast(device_segment_offsets_.data()),
    thrust::raw_pointer_cast(device_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()));

  device_sort_workspace_ =
    thrust::device_vector<std::uint8_t, MemoryPoolAllocator<std::uint8_t>>(allocator_uint8);

  device_transformed_points_.resize(num_organized_points_);
  device_crop_mask_.resize(num_organized_points_);
  device_ring_outlier_mask_.resize(num_organized_points_);
  device_indices_.resize(num_organized_points_);

  preallocateOutput();
}

void CudaPointcloudPreprocessor::setCropBoxParameters(
  const std::vector<CropBoxParameters> & crop_box_parameters)
{
  host_crop_box_structs_ = crop_box_parameters;
  device_crop_box_structs_ = host_crop_box_structs_;
}

void CudaPointcloudPreprocessor::setRingOutlierFilterParameters(
  const RingOutlierFilterParameters & ring_outlier_parameters)
{
  ring_outlier_parameters_ = ring_outlier_parameters;
}

void CudaPointcloudPreprocessor::set3DUndistortion(bool use_3d_undistortion)
{
  use_3d_undistortion_ = use_3d_undistortion;
}

void CudaPointcloudPreprocessor::preallocateOutput()
{
  output_pointcloud_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output_pointcloud_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
    num_rings_ * max_points_per_ring_ * sizeof(OutputPointType));
}

void CudaPointcloudPreprocessor::setupTwist2DStructs(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint64_t pointcloud_stamp_nsec, const std::uint32_t first_point_rel_stamp_nsec)
{
  host_twist_2d_structs_.clear();
  host_twist_2d_structs_.reserve(twist_queue.size() + angular_velocity_queue.size());

  // Twist preprocessing
  float cum_x = 0;
  float cum_y = 0;
  float cum_theta = 0;
  // All time stamps from now on are in nsec from the "beginning of the pointcloud"
  std::uint32_t last_stamp_nsec = first_point_rel_stamp_nsec;

  std::size_t twist_index = 0;
  std::size_t angular_velocity_index = 0;

  for (; twist_index < twist_queue.size() ||
         angular_velocity_index < angular_velocity_queue.size();) {
    std::uint64_t twist_stamp, input_twist_global_stamp_nsec, angular_velocity_global_stamp_nsec;
    float v_x, v_theta;

    if (twist_index < twist_queue.size()) {
      input_twist_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.sec) +
        static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.nanosec);
      v_x = twist_queue[twist_index].twist.twist.linear.x;
    } else {
      input_twist_global_stamp_nsec = std::numeric_limits<std::uint64_t>::max();
      v_x = 0.0;
    }

    if (angular_velocity_index < angular_velocity_queue.size()) {
      angular_velocity_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(
                          angular_velocity_queue[angular_velocity_index].header.stamp.sec) +
        static_cast<std::uint64_t>(
          angular_velocity_queue[angular_velocity_index].header.stamp.nanosec);
      v_theta = angular_velocity_queue[angular_velocity_index].vector.z;
    } else {
      angular_velocity_global_stamp_nsec = std::numeric_limits<std::uint64_t>::max();
      v_theta = 0.0;
    }

    if (input_twist_global_stamp_nsec < angular_velocity_global_stamp_nsec) {
      twist_stamp = input_twist_global_stamp_nsec;
      twist_index++;
    } else if (input_twist_global_stamp_nsec > angular_velocity_global_stamp_nsec) {
      twist_stamp = angular_velocity_global_stamp_nsec;
      angular_velocity_index++;
    } else {
      twist_index++;
      angular_velocity_index++;
    }

    TwistStruct2D twist;
    twist.cum_x = cum_x;
    twist.cum_y = cum_y;
    twist.cum_theta = cum_theta;

    std::uint64_t twist_global_stamp_nsec = twist_stamp;
    assert(twist_global_stamp_nsec > pointcloud_stamp_nsec);  // by construction
    std::uint32_t twist_from_pointcloud_start_nsec =
      twist_global_stamp_nsec - pointcloud_stamp_nsec;

    twist.stamp_nsec = twist_from_pointcloud_start_nsec;
    twist.v_x = v_x;
    twist.v_theta = v_theta;
    twist.last_stamp_nsec = last_stamp_nsec;
    host_twist_2d_structs_.push_back(twist);

    double dt_seconds = 1e-9 * (twist.stamp_nsec - last_stamp_nsec);
    last_stamp_nsec = twist.stamp_nsec;
    cum_theta += v_theta * dt_seconds;
    float d = twist.v_x * dt_seconds;
    cum_x += d * cos(cum_theta);
    cum_y += d * sin(cum_theta);
  }

  // Copy to device
  device_twist_2d_structs_.resize(host_twist_2d_structs_.size());
  cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_twist_2d_structs_.data()), host_twist_2d_structs_.data(),
    host_twist_2d_structs_.size() * sizeof(TwistStruct2D), cudaMemcpyHostToDevice, stream_);
}

void CudaPointcloudPreprocessor::setupTwist3DStructs(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint64_t pointcloud_stamp_nsec, const std::uint32_t first_point_rel_stamp_nsec)
{
  // Twist preprocessing
  host_twist_3d_structs_.clear();
  host_twist_3d_structs_.reserve(twist_queue.size() + angular_velocity_queue.size());

  Eigen::Matrix4f cum_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector3f v = Eigen::Vector3f::Zero();
  Eigen::Vector3f w = Eigen::Vector3f::Zero();

  // All time stamps from now on are in nsec from the "beginning of the pointcloud"
  std::uint32_t last_stamp_nsec = first_point_rel_stamp_nsec;

  std::size_t twist_index = 0;
  std::size_t angular_velocity_index = 0;

  for (; twist_index < twist_queue.size() ||
         angular_velocity_index < angular_velocity_queue.size();) {
    std::uint64_t twist_stamp, input_twist_global_stamp_nsec, angular_velocity_global_stamp_nsec;

    if (twist_index < twist_queue.size()) {
      input_twist_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.sec) +
        static_cast<std::uint64_t>(twist_queue[twist_index].header.stamp.nanosec);
      v.x() = twist_queue[twist_index].twist.twist.linear.x;
      v.y() = twist_queue[twist_index].twist.twist.linear.y;
      v.z() = twist_queue[twist_index].twist.twist.linear.z;
    } else {
      input_twist_global_stamp_nsec = std::numeric_limits<uint64_t>::max();
      v = Eigen::Vector3f::Zero();
    }

    if (angular_velocity_index < angular_velocity_queue.size()) {
      angular_velocity_global_stamp_nsec =
        1'000'000'000 * static_cast<std::uint64_t>(
                          angular_velocity_queue[angular_velocity_index].header.stamp.sec) +
        static_cast<std::uint64_t>(
          angular_velocity_queue[angular_velocity_index].header.stamp.nanosec);
      w.x() = angular_velocity_queue[angular_velocity_index].vector.x;
      w.y() = angular_velocity_queue[angular_velocity_index].vector.y;
      w.z() = angular_velocity_queue[angular_velocity_index].vector.z;
    } else {
      angular_velocity_global_stamp_nsec = std::numeric_limits<std::uint64_t>::max();
      w = Eigen::Vector3f::Zero();
    }

    if (input_twist_global_stamp_nsec < angular_velocity_global_stamp_nsec) {
      twist_stamp = input_twist_global_stamp_nsec;
      twist_index++;
    } else if (input_twist_global_stamp_nsec > angular_velocity_global_stamp_nsec) {
      twist_stamp = angular_velocity_global_stamp_nsec;
      angular_velocity_index++;
    } else {
      twist_index++;
      angular_velocity_index++;
    }

    TwistStruct3D twist;

    Eigen::Map<Eigen::Matrix4f> cum_transform_buffer_map(twist.cum_transform_buffer);
    Eigen::Map<Eigen::Vector3f> v_map(twist.v);
    Eigen::Map<Eigen::Vector3f> w_map(twist.w);
    cum_transform_buffer_map = cum_transform;

    std::uint64_t twist_global_stamp_nsec = twist_stamp;
    assert(twist_global_stamp_nsec > pointcloud_stamp_nsec);  // by construction
    std::uint32_t twist_from_pointcloud_start_nsec =
      twist_global_stamp_nsec - pointcloud_stamp_nsec;

    twist.stamp_nsec = twist_from_pointcloud_start_nsec;
    v_map = v;
    w_map = w;
    twist.last_stamp_nsec = last_stamp_nsec;
    host_twist_3d_structs_.push_back(twist);

    double dt_seconds = 1e-9 * (twist.stamp_nsec - last_stamp_nsec);
    last_stamp_nsec = twist.stamp_nsec;

    auto delta_transform = transformationMatrixFromVelocity(v, w, dt_seconds);
    cum_transform = cum_transform * delta_transform;
  }

  // Copy to device
  device_twist_3d_structs_.resize(host_twist_3d_structs_.size());
  cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_twist_3d_structs_.data()), host_twist_3d_structs_.data(),
    host_twist_3d_structs_.size() * sizeof(TwistStruct3D), cudaMemcpyHostToDevice, stream_);
}

std::size_t CudaPointcloudPreprocessor::querySortWorkspace(
  int num_items, int num_segments, int * offsets_device, std::uint32_t * keys_in_device,
  std::uint32_t * keys_out_device)
{
  // Determine temporary device storage requirements
  void * temp_storage = nullptr;
  size_t temp_storage_bytes = 0;
  cub::DeviceSegmentedRadixSort::SortKeys(
    temp_storage, temp_storage_bytes, keys_in_device, keys_out_device, num_items, num_segments,
    offsets_device, offsets_device + 1);

  return temp_storage_bytes;
}

void CudaPointcloudPreprocessor::organizePointcloud()
{
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_ring_index_.data()), 0, num_rings_ * sizeof(std::int32_t),
    stream_);
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_indexes_tensor_.data()), 0xFF,
    num_organized_points_ * sizeof(std::uint32_t), stream_);

  if (num_raw_points_ == 0) {
    return;
  }

  const int raw_points_blocks_per_grid =
    (num_raw_points_ + threads_per_block_ - 1) / threads_per_block_;

  organizeKernel<<<raw_points_blocks_per_grid, threads_per_block_, 0, stream_>>>(
    thrust::raw_pointer_cast(device_input_points_.data()),
    thrust::raw_pointer_cast(device_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_ring_index_.data()), num_rings_,
    thrust::raw_pointer_cast(device_max_ring_.data()), max_points_per_ring_,
    thrust::raw_pointer_cast(device_max_points_per_ring_.data()), num_raw_points_);

  std::int32_t max_ring_value;
  std::int32_t max_points_per_ring;

  cudaMemcpyAsync(
    &max_ring_value, thrust::raw_pointer_cast(device_max_ring_.data()), sizeof(std::int32_t),
    cudaMemcpyDeviceToHost, stream_);
  cudaMemcpyAsync(
    &max_points_per_ring, thrust::raw_pointer_cast(device_max_points_per_ring_.data()),
    sizeof(std::int32_t), cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  if (max_ring_value >= num_rings_ || max_points_per_ring > max_points_per_ring_) {
    num_rings_ = max_ring_value + 1;
    max_points_per_ring_ = std::max((max_points_per_ring + 511) / 512 * 512, 512);
    num_organized_points_ = num_rings_ * max_points_per_ring_;

    device_ring_index_.resize(num_rings_);
    device_indexes_tensor_.resize(num_organized_points_);
    device_sorted_indexes_tensor_.resize(num_organized_points_);
    device_segment_offsets_.resize(num_rings_ + 1);
    device_organized_points_.resize(num_organized_points_);

    device_transformed_points_.resize(num_organized_points_);
    device_crop_mask_.resize(num_organized_points_);
    device_ring_outlier_mask_.resize(num_organized_points_);
    device_indices_.resize(num_organized_points_);

    preallocateOutput();

    std::vector<std::int32_t> segment_offsets_host(num_rings_ + 1);
    for (std::size_t i = 0; i < num_rings_ + 1; i++) {
      segment_offsets_host[i] = i * max_points_per_ring_;
    }

    cudaMemcpyAsync(
      thrust::raw_pointer_cast(device_segment_offsets_.data()), segment_offsets_host.data(),
      (num_rings_ + 1) * sizeof(std::int32_t), cudaMemcpyHostToDevice, stream_);

    cudaMemsetAsync(
      thrust::raw_pointer_cast(device_ring_index_.data()), 0, num_rings_ * sizeof(std::int32_t),
      stream_);
    cudaMemsetAsync(
      thrust::raw_pointer_cast(device_indexes_tensor_.data()), 0xFF,
      num_organized_points_ * sizeof(std::int32_t), stream_);

    sort_workspace_bytes_ = querySortWorkspace(
      num_organized_points_, num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()));
    device_sort_workspace_.resize(sort_workspace_bytes_);

    organizeKernel<<<raw_points_blocks_per_grid, threads_per_block_, 0, stream_>>>(
      thrust::raw_pointer_cast(device_input_points_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_ring_index_.data()), num_rings_,
      thrust::raw_pointer_cast(device_max_ring_.data()), max_points_per_ring_,
      thrust::raw_pointer_cast(device_max_points_per_ring_.data()), num_raw_points_);
  }

  if (num_organized_points_ == num_rings_) {
    cudaMemcpyAsync(
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      num_organized_points_ * sizeof(std::uint32_t), cudaMemcpyDeviceToDevice, stream_);
  } else {
    cub::DeviceSegmentedRadixSort::SortKeys(
      reinterpret_cast<void *>(thrust::raw_pointer_cast(device_sort_workspace_.data())),
      sort_workspace_bytes_, thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()), num_organized_points_,
      num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
      thrust::raw_pointer_cast(device_segment_offsets_.data()) + 1, 0, sizeof(std::uint32_t) * 8,
      stream_);
  }

  const int organized_points_blocks_per_grid =
    (num_organized_points_ + threads_per_block_ - 1) / threads_per_block_;
  gatherKernel<<<organized_points_blocks_per_grid, threads_per_block_, 0, stream_>>>(
    thrust::raw_pointer_cast(device_input_points_.data()),
    thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_organized_points_.data()), num_rings_, max_points_per_ring_);
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaPointcloudPreprocessor::process(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr,
  const geometry_msgs::msg::TransformStamped & transform_msg,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint32_t first_point_rel_stamp_nsec)
{
  auto frame_id = input_pointcloud_msg_ptr->header.frame_id;
  num_raw_points_ = input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height;
  num_organized_points_ = num_rings_ * max_points_per_ring_;

  if (num_raw_points_ > device_input_points_.size()) {
    std::size_t new_capacity = (num_raw_points_ + 1024) / 1024 * 1024;
    device_input_points_.resize(new_capacity);
  }

  cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_input_points_.data()), input_pointcloud_msg_ptr->data.data(),
    num_raw_points_ * sizeof(InputPointType), cudaMemcpyHostToDevice, stream_);

  cudaStreamSynchronize(stream_);

  organizePointcloud();

  tf2::Quaternion rotation_quaternion(
    transform_msg.transform.rotation.x, transform_msg.transform.rotation.y,
    transform_msg.transform.rotation.z, transform_msg.transform.rotation.w);
  tf2::Matrix3x3 rotation_matrix;
  rotation_matrix.setRotation(rotation_quaternion);

  TransformStruct transform_struct;
  transform_struct.x = static_cast<float>(transform_msg.transform.translation.x);
  transform_struct.y = static_cast<float>(transform_msg.transform.translation.y);
  transform_struct.z = static_cast<float>(transform_msg.transform.translation.z);
  transform_struct.m11 = static_cast<float>(rotation_matrix[0][0]);
  transform_struct.m12 = static_cast<float>(rotation_matrix[0][1]);
  transform_struct.m13 = static_cast<float>(rotation_matrix[0][2]);
  transform_struct.m21 = static_cast<float>(rotation_matrix[1][0]);
  transform_struct.m22 = static_cast<float>(rotation_matrix[1][1]);
  transform_struct.m23 = static_cast<float>(rotation_matrix[1][2]);
  transform_struct.m31 = static_cast<float>(rotation_matrix[2][0]);
  transform_struct.m32 = static_cast<float>(rotation_matrix[2][1]);
  transform_struct.m33 = static_cast<float>(rotation_matrix[2][2]);

  // Twist preprocessing
  std::uint64_t pointcloud_stamp_nsec = 1'000'000'000 * input_pointcloud_msg_ptr->header.stamp.sec +
                                        input_pointcloud_msg_ptr->header.stamp.nanosec;

  if (use_3d_undistortion_) {
    setupTwist3DStructs(
      twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, first_point_rel_stamp_nsec);
  } else {
    setupTwist2DStructs(
      twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, first_point_rel_stamp_nsec);
  }

  // check_error(frame_id, "After setup");

  // Obtain raw pointers for the kernels
  TwistStruct2D * device_twist_2d_structs =
    thrust::raw_pointer_cast(device_twist_2d_structs_.data());
  TwistStruct3D * device_twist_3d_structs =
    thrust::raw_pointer_cast(device_twist_3d_structs_.data());
  InputPointType * device_transformed_points =
    thrust::raw_pointer_cast(device_transformed_points_.data());
  std::uint32_t * device_crop_mask = thrust::raw_pointer_cast(device_crop_mask_.data());
  std::uint32_t * device_ring_outlier_mask =
    thrust::raw_pointer_cast(device_ring_outlier_mask_.data());
  std::uint32_t * device_indices = thrust::raw_pointer_cast(device_indices_.data());

  const int blocks_per_grid = (num_organized_points_ + threads_per_block_ - 1) / threads_per_block_;

  transformPointsKernel<<<blocks_per_grid, threads_per_block_, 0, stream_>>>(
    thrust::raw_pointer_cast(device_organized_points_.data()), device_transformed_points,
    num_organized_points_, transform_struct);

  int crop_box_blocks_per_grid = std::min(blocks_per_grid, max_blocks_per_grid_);
  if (host_crop_box_structs_.size() > 0) {
    cropBoxKernel<<<crop_box_blocks_per_grid, threads_per_block_, 0, stream_>>>(
      device_transformed_points, device_crop_mask, num_organized_points_,
      thrust::raw_pointer_cast(device_crop_box_structs_.data()), host_crop_box_structs_.size());
  } else {
    thrust::fill(thrust::device, device_crop_mask, device_crop_mask + num_organized_points_, 1);
  }

  if (use_3d_undistortion_ && device_twist_3d_structs_.size() > 0) {
    undistort3dKernel<<<blocks_per_grid, threads_per_block_, 0, stream_>>>(
      device_transformed_points, num_organized_points_, device_twist_3d_structs,
      device_twist_3d_structs_.size());
  } else if (!use_3d_undistortion_ && device_twist_2d_structs_.size() > 0) {
    undistort2dKernel<<<blocks_per_grid, threads_per_block_, 0, stream_>>>(
      device_transformed_points, num_organized_points_, device_twist_2d_structs,
      device_twist_2d_structs_.size());
  }

  ringOutlierFilterKernel<<<blocks_per_grid, threads_per_block_, 0, stream_>>>(
    device_transformed_points, device_ring_outlier_mask, num_rings_, max_points_per_ring_,
    ring_outlier_parameters_.distance_ratio,
    ring_outlier_parameters_.object_length_threshold *
      ring_outlier_parameters_.object_length_threshold,
    ring_outlier_parameters_.num_points_threshold);

  combineMasksKernel<<<blocks_per_grid, threads_per_block_, 0, stream_>>>(
    device_crop_mask, device_ring_outlier_mask, num_organized_points_, device_ring_outlier_mask);

  thrust::inclusive_scan(
    thrust::device, device_ring_outlier_mask, device_ring_outlier_mask + num_organized_points_,
    device_indices);

  std::uint32_t num_output_points;
  cudaMemcpyAsync(
    &num_output_points, device_indices + num_organized_points_ - 1, sizeof(std::uint32_t),
    cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  if (num_output_points > 0) {
    extractInputPointsToOutputPoints_indicesKernel<<<
      blocks_per_grid, threads_per_block_, 0, stream_>>>(
      device_transformed_points, device_ring_outlier_mask, device_indices, num_organized_points_,
      reinterpret_cast<OutputPointType *>(output_pointcloud_ptr_->data.get()));
  }

  cudaStreamSynchronize(stream_);

  // Copy the transformed points back
  output_pointcloud_ptr_->row_step = num_output_points * sizeof(OutputPointType);
  output_pointcloud_ptr_->width = num_output_points;
  output_pointcloud_ptr_->height = 1;

  output_pointcloud_ptr_->fields = point_fields_;
  output_pointcloud_ptr_->is_dense = true;
  output_pointcloud_ptr_->is_bigendian = input_pointcloud_msg_ptr->is_bigendian;
  output_pointcloud_ptr_->point_step = sizeof(OutputPointType);
  output_pointcloud_ptr_->header.stamp = input_pointcloud_msg_ptr->header.stamp;

  return std::move(output_pointcloud_ptr_);
}

}  // namespace autoware::cuda_pointcloud_preprocessor
