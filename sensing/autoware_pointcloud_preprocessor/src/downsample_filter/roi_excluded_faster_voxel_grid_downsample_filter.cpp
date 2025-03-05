// Copyright 2025 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/downsample_filter/roi_excluded_faster_voxel_grid_downsample_filter.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <cfloat>
#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

RoiExcludedFasterVoxelGridDownsampleFilter::RoiExcludedFasterVoxelGridDownsampleFilter()
{
  offset_initialized_ = false;
}

void RoiExcludedFasterVoxelGridDownsampleFilter::set_voxel_size(
  float voxel_size_x, float voxel_size_y, float voxel_size_z)
{
  inverse_voxel_size_ =
    Eigen::Array3f::Ones() / Eigen::Array3f(voxel_size_x, voxel_size_y, voxel_size_z);
}

void RoiExcludedFasterVoxelGridDownsampleFilter::set_field_offsets(
  const PointCloud2ConstPtr & input, const rclcpp::Logger & logger)
{
  x_offset_ = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  y_offset_ = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  z_offset_ = input->fields[pcl::getFieldIndex(*input, "z")].offset;
  intensity_index_ = pcl::getFieldIndex(*input, "intensity");

  if (
    intensity_index_ < 0 ||
    input->fields[intensity_index_].datatype != sensor_msgs::msg::PointField::UINT8) {
    RCLCPP_ERROR(logger, "No intensity field in the input or intensity type is not UINT8.");
  }

  intensity_offset_ = (intensity_index_ != -1) ? input->fields[intensity_index_].offset : -1;
  offset_initialized_ = true;
}

void RoiExcludedFasterVoxelGridDownsampleFilter::set_roi(
  float x_min, float x_max, float y_min, float y_max)
{
  roi_x_min_ = x_min;
  roi_x_max_ = x_max;
  roi_y_min_ = y_min;
  roi_y_max_ = y_max;
}

Eigen::Vector4f RoiExcludedFasterVoxelGridDownsampleFilter::get_point_from_global_offset(
  const PointCloud2ConstPtr & input, size_t global_offset)
{
  float intensity = 0.0f;
  if (intensity_index_ >= 0) {
    intensity = static_cast<float>(
      *(reinterpret_cast<const uint8_t *>(&input->data[global_offset + intensity_offset_])));
  }
  return Eigen::Vector4f(
    *(reinterpret_cast<const float *>(&input->data[global_offset + x_offset_])),
    *(reinterpret_cast<const float *>(&input->data[global_offset + y_offset_])),
    *(reinterpret_cast<const float *>(&input->data[global_offset + z_offset_])), intensity);
}

bool RoiExcludedFasterVoxelGridDownsampleFilter::get_min_max_voxel(
  const PointCloud2ConstPtr & input, Eigen::Vector3i & min_voxel, Eigen::Vector3i & max_voxel)
{
  Eigen::Vector3f min_point, max_point;
  min_point.setConstant(FLT_MAX);
  max_point.setConstant(-FLT_MAX);
  for (size_t offset = 0; offset + input->point_step <= input->data.size();
       offset += input->point_step) {
    Eigen::Vector4f point = get_point_from_global_offset(input, offset);
    if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
      min_point = min_point.cwiseMin(point.head<3>());
      max_point = max_point.cwiseMax(point.head<3>());
    }
  }

  if (
    ((static_cast<std::int64_t>((max_point[0] - min_point[0]) * inverse_voxel_size_[0]) + 1) *
     (static_cast<std::int64_t>((max_point[1] - min_point[1]) * inverse_voxel_size_[1]) + 1) *
     (static_cast<std::int64_t>((max_point[2] - min_point[2]) * inverse_voxel_size_[2]) + 1)) >
    static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
    return false;
  }
  min_voxel[0] = static_cast<int>(std::floor(min_point[0] * inverse_voxel_size_[0]));
  min_voxel[1] = static_cast<int>(std::floor(min_point[1] * inverse_voxel_size_[1]));
  min_voxel[2] = static_cast<int>(std::floor(min_point[2] * inverse_voxel_size_[2]));
  max_voxel[0] = static_cast<int>(std::floor(max_point[0] * inverse_voxel_size_[0]));
  max_voxel[1] = static_cast<int>(std::floor(max_point[1] * inverse_voxel_size_[1]));
  max_voxel[2] = static_cast<int>(std::floor(max_point[2] * inverse_voxel_size_[2]));
  return true;
}

std::unordered_map<uint32_t, RoiExcludedFasterVoxelGridDownsampleFilter::Centroid>
RoiExcludedFasterVoxelGridDownsampleFilter::calc_centroids_each_voxel(
  const PointCloud2ConstPtr & input, const Eigen::Vector3i & max_voxel,
  const Eigen::Vector3i & min_voxel)
{
  std::unordered_map<uint32_t, Centroid> voxel_centroid_map;
  Eigen::Vector3i div_b = max_voxel - min_voxel + Eigen::Vector3i::Ones();
  Eigen::Vector3i div_b_mul(1, div_b[0], div_b[0] * div_b[1]);

  for (size_t offset = 0; offset + input->point_step <= input->data.size();
       offset += input->point_step) {
    Eigen::Vector4f point = get_point_from_global_offset(input, offset);
    if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
      int ijk0 = static_cast<int>(std::floor(point[0] * inverse_voxel_size_[0]) - min_voxel[0]);
      int ijk1 = static_cast<int>(std::floor(point[1] * inverse_voxel_size_[1]) - min_voxel[1]);
      int ijk2 = static_cast<int>(std::floor(point[2] * inverse_voxel_size_[2]) - min_voxel[2]);
      uint32_t voxel_id = ijk0 * div_b_mul[0] + ijk1 * div_b_mul[1] + ijk2 * div_b_mul[2];
      if (voxel_centroid_map.find(voxel_id) == voxel_centroid_map.end()) {
        voxel_centroid_map[voxel_id] = Centroid(point[0], point[1], point[2], point[3]);
      } else {
        voxel_centroid_map[voxel_id].add_point(point[0], point[1], point[2], point[3]);
      }
    }
  }
  return voxel_centroid_map;
}

void RoiExcludedFasterVoxelGridDownsampleFilter::copy_centroids_to_output(
  std::unordered_map<uint32_t, Centroid> & voxel_centroid_map, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  size_t output_offset = 0;
  for (const auto & pair : voxel_centroid_map) {
    Eigen::Vector4f centroid = pair.second.calc_centroid();
    if (transform_info.need_transform) {
      centroid = transform_info.eigen_transform * centroid;
    }
    *reinterpret_cast<float *>(&output.data[output_offset + x_offset_]) = centroid[0];
    *reinterpret_cast<float *>(&output.data[output_offset + y_offset_]) = centroid[1];
    *reinterpret_cast<float *>(&output.data[output_offset + z_offset_]) = centroid[2];
    *reinterpret_cast<uint8_t *>(&output.data[output_offset + intensity_offset_]) =
      static_cast<uint8_t>(centroid[3]);
    output_offset += output.point_step;
  }
}

void RoiExcludedFasterVoxelGridDownsampleFilter::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info,
  const rclcpp::Logger & logger)
{
  if (!offset_initialized_) {
    set_field_offsets(input, logger);
  }
  // Partition the input into points for voxel filtering and points to pass directly.
  std::vector<uint8_t> filtered_buffer;
  std::vector<uint8_t> direct_buffer;
  const size_t point_step = input->point_step;
  for (size_t offset = 0; offset + point_step <= input->data.size(); offset += point_step) {
    Eigen::Vector4f point = get_point_from_global_offset(input, offset);
    // If point is inside ROI, bypass filtering.
    if (
      (point[0] >= roi_x_min_ && point[0] <= roi_x_max_) &&
      (point[1] >= roi_y_min_ && point[1] <= roi_y_max_)) {
      direct_buffer.insert(
        direct_buffer.end(), input->data.begin() + offset,
        input->data.begin() + offset + point_step);
    } else {
      filtered_buffer.insert(
        filtered_buffer.end(), input->data.begin() + offset,
        input->data.begin() + offset + point_step);
    }
  }

  // Build a temporary cloud for filtered points.
  PointCloud2 filtered_cloud;
  filtered_cloud.header = input->header;
  filtered_cloud.fields = input->fields;
  filtered_cloud.is_bigendian = input->is_bigendian;
  filtered_cloud.point_step = point_step;
  filtered_cloud.data = filtered_buffer;
  filtered_cloud.width = filtered_buffer.size() / point_step;
  filtered_cloud.height = 1;  // assume organized as unorganized

  // Process voxel filtering on the filtered_cloud if not empty.
  PointCloud2 downsampled_filtered;
  if (!filtered_cloud.data.empty()) {
    Eigen::Vector3i min_voxel, max_voxel;
    if (!get_min_max_voxel(std::make_shared<PointCloud2>(filtered_cloud), min_voxel, max_voxel)) {
      RCLCPP_ERROR(logger, "Voxel size too small or data error in filtered cloud.");
      downsampled_filtered = filtered_cloud;  // fallback to unfiltered
    } else {
      auto voxel_map = calc_centroids_each_voxel(
        std::make_shared<PointCloud2>(filtered_cloud), max_voxel, min_voxel);
      size_t num_points = voxel_map.size();
      downsampled_filtered.header = filtered_cloud.header;
      downsampled_filtered.fields = filtered_cloud.fields;
      downsampled_filtered.is_bigendian = filtered_cloud.is_bigendian;
      downsampled_filtered.point_step = point_step;
      downsampled_filtered.width = num_points;
      downsampled_filtered.height = 1;
      downsampled_filtered.row_step = num_points * point_step;
      downsampled_filtered.data.resize(downsampled_filtered.row_step);
      copy_centroids_to_output(voxel_map, downsampled_filtered, transform_info);
    }
  }

  // Create final output by concatenating downsampled filtered points and direct points.
  size_t num_downsampled = downsampled_filtered.data.size() / point_step;
  size_t num_direct = direct_buffer.size() / point_step;
  size_t total_points = num_downsampled + num_direct;

  output.header = input->header;
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = point_step;
  output.width = total_points;
  output.height = 1;
  output.row_step = total_points * point_step;
  output.data.resize(output.row_step);

  // Copy voxel filtered points first.
  std::copy(
    downsampled_filtered.data.begin(), downsampled_filtered.data.end(), output.data.begin());
  // Then append direct points.
  std::copy(
    direct_buffer.begin(), direct_buffer.end(),
    output.data.begin() + downsampled_filtered.data.size());
}

}  // namespace autoware::pointcloud_preprocessor
