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

#include "lidar_transfusion/preprocess/voxel_generator.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lidar_transfusion
{

VoxelGenerator::VoxelGenerator(
  const DensificationParam & densification_param, const TransfusionConfig & config,
  cudaStream_t & stream)
: config_(config), stream_(stream)
{
  pd_ptr_ = std::make_unique<PointCloudDensification>(densification_param);
  pre_ptr_ = std::make_unique<PreprocessCuda>(config_, stream_);
  cloud_data_d_ = cuda::make_unique<unsigned char[]>(config_.cloud_capacity_ * MAX_CLOUD_STEP_SIZE);
  points_.resize(config_.cloud_capacity_ * config_.num_point_feature_size_);
  affine_past2current_d_ = cuda::make_unique<float[]>(AFF_MAT_SIZE);
}

bool VoxelGenerator::enqueuePointCloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer)
{
  return pd_ptr_->enqueuePointCloud(input_pointcloud_msg, tf_buffer);
}

std::size_t VoxelGenerator::generateSweepPoints(
  const sensor_msgs::msg::PointCloud2 & msg, cuda::unique_ptr<float[]> & points_d)
{
  if (!is_initialized_) {
    initCloudInfo(msg);
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("lidar_transfusion"),
      "PointCloud2 msg information:"
        << "\nx offset: " << cloud_info_.x_offset << "\ny offset: " << cloud_info_.y_offset
        << "\nz offset: " << cloud_info_.z_offset
        << "\nintensity offset: " << cloud_info_.intensity_offset
        << "\nx datatype: " << static_cast<uint32_t>(cloud_info_.x_datatype)
        << "\ny datatype: " << static_cast<uint32_t>(cloud_info_.y_datatype)
        << "\nz datatype: " << static_cast<uint32_t>(cloud_info_.z_datatype)
        << "\nintensity datatype: " << static_cast<uint32_t>(cloud_info_.intensity_datatype)
        << "\npoint step: " << cloud_info_.point_step
        << "\nis bigendian: " << cloud_info_.is_bigendian);
  }

  Eigen::Vector3f point_current, point_past;
  size_t points_agg{};

  for (auto pc_cache_iter = pd_ptr_->getPointCloudCacheIter(); !pd_ptr_->isCacheEnd(pc_cache_iter);
       pc_cache_iter++) {
    if (points_agg >= config_.cloud_capacity_) {
      break;
    }
    auto pc_msg = pc_cache_iter->pointcloud_msg;
    auto affine_past2current =
      pd_ptr_->getAffineWorldToCurrent() * pc_cache_iter->affine_past2world;
    float time_lag = static_cast<float>(
      pd_ptr_->getCurrentTimestamp() - rclcpp::Time(pc_msg.header.stamp).seconds());

#ifdef HOST_PROCESSING
    for (sensor_msgs::PointCloud2ConstIterator<float> x_iter(msg, "x"), y_iter(msg, "y"),
         z_iter(msg, "z"), intensity_iter(msg, "intensity");
         x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter, ++intensity_iter) {
      if (points_agg >= config_.cloud_capacity_) {
        break;
      }
      point_past << *x_iter, *y_iter, *z_iter;
      point_current = affine_past2current * point_past;

      points_.at(points_agg * config_.num_point_feature_size_) = point_current.x();
      points_.at(points_agg * config_.num_point_feature_size_ + 1) = point_current.y();
      points_.at(points_agg * config_.num_point_feature_size_ + 2) = point_current.z();
      points_.at(points_agg * config_.num_point_feature_size_ + 3) = *intensity_iter;
      points_.at(points_agg * config_.num_point_feature_size_ + 4) = time_lag;
      ++points_agg;
    }
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    points_d.get(), points_.data(), points_agg * config_.num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

#else
    size_t points_size = pc_msg.width * pc_msg.height;
    cuda::clear_async(cloud_data_d_.get(), config_.cloud_capacity_ * MAX_CLOUD_STEP_SIZE, stream_);
    cuda::clear_async(affine_past2current_d_.get(), AFF_MAT_SIZE, stream_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      cloud_data_d_.get(), pc_msg.data.data(), pc_msg.data.size() * sizeof(unsigned char),
      cudaMemcpyHostToDevice, stream_));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      affine_past2current_d_.get(), affine_past2current.data(), AFF_MAT_SIZE * sizeof(float),
      cudaMemcpyHostToDevice, stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    pre_ptr_->generateVoxelsInput_launch(
      cloud_data_d_.get(), cloud_info_, points_agg, points_size, time_lag,
      affine_past2current_d_.get(), points_d.get());
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    points_agg += points_size;
  }
#endif

  return std::min(points_agg, config_.cloud_capacity_);
}

void VoxelGenerator::initCloudInfo(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::tie(cloud_info_.x_offset, cloud_info_.x_datatype, cloud_info_.x_size) =
    getFieldInfo(msg, "x");
  std::tie(cloud_info_.y_offset, cloud_info_.y_datatype, cloud_info_.y_size) =
    getFieldInfo(msg, "y");
  std::tie(cloud_info_.z_offset, cloud_info_.z_datatype, cloud_info_.z_size) =
    getFieldInfo(msg, "z");
  std::tie(
    cloud_info_.intensity_offset, cloud_info_.intensity_datatype, cloud_info_.intensity_size) =
    getFieldInfo(msg, "intensity");
  cloud_info_.point_step = msg.point_step;
  cloud_info_.is_bigendian = msg.is_bigendian;
  is_initialized_ = true;
}

std::tuple<const uint32_t, const uint8_t, const uint8_t> VoxelGenerator::getFieldInfo(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name)
{
  for (const auto & field : msg.fields) {
    if (field.name == field_name) {
      if (field.count == 1) {
        return std::make_tuple(field.offset, field.datatype, datatype2size.at(field.datatype));
      } else {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("lidar_transfusion"),
          "Unsupported field for " << field_name << ". [Actual / Supported] count: " << field.count
                                   << " / 1.");
      }
    }
  }
  throw std::runtime_error("Incompatible point cloud message for field: " + field_name);
}
}  // namespace lidar_transfusion
