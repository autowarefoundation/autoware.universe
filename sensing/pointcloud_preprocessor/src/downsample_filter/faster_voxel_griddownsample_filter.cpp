// Copyright 2020 Tier IV, Inc.
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

#include "pointcloud_preprocessor/downsample_filter/faster_voxel_grid_dowmsample_filter.hpp"

pointcloud_preprocessor::FasterVoxelGridDownsampleFilter::FasterVoxelGridDownsampleFilter()
{
  pcl::for_each_type<typename pcl::traits::fieldList<pcl::PointXYZ>::type>(
    pcl::detail::FieldAdder<pcl::PointXYZ>(xyz_fields_));
}

void pointcloud_preprocessor::FasterVoxelGridDownsampleFilter::set_voxel_size(
  float voxel_size_x_, float voxel_size_y_, float voxel_size_z_)
{
  inverse_voxel_size_[0] = 1.0f / voxel_size_x_;
  inverse_voxel_size_[1] = 1.0f / voxel_size_y_;
  inverse_voxel_size_[2] = 1.0f / voxel_size_z_;
}

void pointcloud_preprocessor::FasterVoxelGridDownsampleFilter::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info,
  const rclcpp::Logger & logger)
{
  int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;
  int intensity_offset = input->fields[pcl::getFieldIndex(*input, "intensity")].offset;

  // Get the minimum and maximum dimensions
  Eigen::Vector3f min_p, max_p;
  min_p.setConstant(FLT_MAX);
  max_p.setConstant(-FLT_MAX);
  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    Eigen::Vector3f point(
      *reinterpret_cast<const float *>(&input->data[global_offset + x_offset]),
      *reinterpret_cast<const float *>(&input->data[global_offset + y_offset]),
      *reinterpret_cast<const float *>(&input->data[global_offset + z_offset]));

    if (std::isfinite(point[0]) && std::isfinite(point[1]), std::isfinite(point[2])) {
      min_p = min_p.cwiseMin(point);
      max_p = max_p.cwiseMax(point);
    }
  }

  // Check that the voxel size is not too small, given the size of the data
  if (
    ((static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_voxel_size_[0]) + 1) *
     (static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_voxel_size_[1]) + 1) *
     (static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_voxel_size_[2]) + 1)) >
    static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
    RCLCPP_ERROR(
      logger,
      "Voxel size is too small for the input dataset. "
      "Integer indices would overflow.");
    output = *input;
    return;
  }

  // Compute the minimum and maximum bounding box values
  Eigen::Vector3f min_b, max_b;
  min_b[0] = static_cast<int>(std::floor(min_p[0] * inverse_voxel_size_[0]));
  max_b[0] = static_cast<int>(std::floor(max_p[0] * inverse_voxel_size_[0]));
  min_b[1] = static_cast<int>(std::floor(min_p[1] * inverse_voxel_size_[1]));
  max_b[1] = static_cast<int>(std::floor(max_p[1] * inverse_voxel_size_[1]));
  min_b[2] = static_cast<int>(std::floor(min_p[2] * inverse_voxel_size_[2]));
  max_b[2] = static_cast<int>(std::floor(max_p[2] * inverse_voxel_size_[2]));

  // Compute the number of divisions needed along all axis
  Eigen::Vector3f div_b = max_b - min_b + Eigen::Vector3f::Ones();

  // Set up the division multiplier
  Eigen::Vector3i div_b_mul(1, div_b[0], div_b[0] * div_b[1]);

  // Storage for mapping voxel coordinates to centroids
  std::unordered_map<uint32_t, Centroid> voxel_centroid_map;
  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    Eigen::Vector3f point(
      *reinterpret_cast<const float *>(&input->data[global_offset + x_offset]),
      *reinterpret_cast<const float *>(&input->data[global_offset + y_offset]),
      *reinterpret_cast<const float *>(&input->data[global_offset + z_offset]));
    if (std::isfinite(point[0]) && std::isfinite(point[1]), std::isfinite(point[2])) {
      int ijk0 = static_cast<int>(
        std::floor(point[0] * inverse_voxel_size_[0]) - static_cast<float>(min_b[0]));
      int ijk1 = static_cast<int>(
        std::floor(point[1] * inverse_voxel_size_[1]) - static_cast<float>(min_b[1]));
      int ijk2 = static_cast<int>(
        std::floor(point[2] * inverse_voxel_size_[2]) - static_cast<float>(min_b[2]));
      uint32_t voxel_id = ijk0 * div_b_mul[0] + ijk1 * div_b_mul[1] + ijk2 * div_b_mul[2];

      if (voxel_centroid_map.find(voxel_id) == voxel_centroid_map.end()) {
        voxel_centroid_map[voxel_id] = Centroid(point[0], point[1], point[2]);
      } else {
        voxel_centroid_map[voxel_id].add_point(point[0], point[1], point[2]);
      }
    }
  }

  // Copy the centroids to the output
  output.row_step = voxel_centroid_map.size() * input->point_step;
  output.data.resize(output.row_step);
  size_t output_data_size = 0;
  for (auto & pair : voxel_centroid_map) {
    pair.second.calc_centroid();
    if (transform_info.need_transform) {
      Eigen::Array4f point = transform_info.eigen_transform *
                             Eigen::Vector4f(pair.second.x, pair.second.y, pair.second.z, 1);
      *reinterpret_cast<float *>(&output.data[output_data_size + x_offset]) = point[0];
      *reinterpret_cast<float *>(&output.data[output_data_size + y_offset]) = point[1];
      *reinterpret_cast<float *>(&output.data[output_data_size + z_offset]) = point[2];
    } else {
      *reinterpret_cast<float *>(&output.data[output_data_size + x_offset]) = pair.second.x;
      *reinterpret_cast<float *>(&output.data[output_data_size + y_offset]) = pair.second.y;
      *reinterpret_cast<float *>(&output.data[output_data_size + z_offset]) = pair.second.z;
    }

    *reinterpret_cast<float *>(&output.data[output_data_size + intensity_offset]) = 1.0f;
    output_data_size += input->point_step;
  }

  // Post processing
  pcl_conversions::fromPCL(xyz_fields_, output.fields);
  output.width = voxel_centroid_map.size();
  output.is_dense = true;  // we filter out invalid points
  output.height = input->height;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.header = input->header;
}