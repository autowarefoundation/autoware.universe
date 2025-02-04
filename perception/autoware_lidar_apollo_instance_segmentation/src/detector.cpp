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

#include "autoware/lidar_apollo_instance_segmentation/detector.hpp"

#include "autoware/lidar_apollo_instance_segmentation/feature_map.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <NvInfer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace lidar_apollo_instance_segmentation
{
LidarApolloInstanceSegmentation::LidarApolloInstanceSegmentation(rclcpp::Node * node)
: node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
{
  int range, width, height;
  bool use_intensity_feature, use_constant_feature;
  std::string onnx_file;
  score_threshold_ = node_->declare_parameter("score_threshold", 0.8);
  range = node_->declare_parameter("range", 60);
  width = node_->declare_parameter("width", 640);
  height = node_->declare_parameter("height", 640);
  onnx_file = node_->declare_parameter("onnx_file", "vls-128.onnx");
  use_intensity_feature = node_->declare_parameter("use_intensity_feature", true);
  use_constant_feature = node_->declare_parameter("use_constant_feature", true);
  target_frame_ = node_->declare_parameter("target_frame", "base_link");
  z_offset_ = node_->declare_parameter<float>("z_offset", -2.0);
  const auto precision = node_->declare_parameter("precision", "fp32");

  trt_common_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    tensorrt_common::TrtCommonConfig(onnx_file, precision));
  if (!trt_common_->setup()) {
    throw std::runtime_error("Failed to setup TensorRT");
  }

  if (node_->declare_parameter("build_only", false)) {
    RCLCPP_INFO(node_->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }

  // GPU memory allocation
  const auto input_dims = trt_common_->getTensorShape(0);
  const auto input_size =
    std::accumulate(input_dims.d + 1, input_dims.d + input_dims.nbDims, 1, std::multiplies<int>());
  input_d_ = autoware::cuda_utils::make_unique<float[]>(input_size);
  const auto output_dims = trt_common_->getTensorShape(1);
  output_size_ = std::accumulate(
    output_dims.d + 1, output_dims.d + output_dims.nbDims, 1, std::multiplies<int>());
  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_size_);
  output_h_ = autoware::cuda_utils::make_unique_host<float[]>(output_size_, cudaHostAllocPortable);

  // feature map generator: pre process
  feature_generator_ = std::make_shared<FeatureGenerator>(
    width, height, range, use_intensity_feature, use_constant_feature);

  // cluster: post process
  cluster2d_ = std::make_shared<Cluster2D>(width, height, range);
}

bool LidarApolloInstanceSegmentation::transformCloud(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & transformed_cloud,
  float z_offset)
{
  // TODO(mitsudome-r): remove conversion once pcl_ros transform are available.
  pcl::PointCloud<pcl::PointXYZI> pcl_input, pcl_transformed_cloud;
  pcl::fromROSMsg(input, pcl_input);

  // transform pointcloud to target_frame
  if (target_frame_ != input.header.frame_id) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform(
        target_frame_, input.header.frame_id, input.header.stamp, std::chrono::milliseconds(500));
      Eigen::Matrix4f affine_matrix =
        tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
      autoware::universe_utils::transformPointCloud(
        pcl_input, pcl_transformed_cloud, affine_matrix);
      transformed_cloud.header.frame_id = target_frame_;
      pcl_transformed_cloud.header.frame_id = target_frame_;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      return false;
    }
  } else {
    pcl_transformed_cloud = pcl_input;
  }

  // move pointcloud z_offset in z axis
  pcl::PointCloud<pcl::PointXYZI> pointcloud_with_z_offset;
  Eigen::Affine3f z_up_translation(Eigen::Translation3f(0, 0, z_offset));
  Eigen::Matrix4f z_up_transform = z_up_translation.matrix();
  autoware::universe_utils::transformPointCloud(
    pcl_transformed_cloud, pcl_transformed_cloud, z_up_transform);

  pcl::toROSMsg(pcl_transformed_cloud, transformed_cloud);

  return true;
}

bool LidarApolloInstanceSegmentation::detectDynamicObjects(
  const sensor_msgs::msg::PointCloud2 & input,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & output)
{
  // move up pointcloud z_offset in z axis
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  transformCloud(input, transformed_cloud, z_offset_);

  // convert from ros to pcl
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_raw_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::fromROSMsg(transformed_cloud, *pcl_pointcloud_raw_ptr);

  auto & pcl_pointcloud_raw = *pcl_pointcloud_raw_ptr;
  pcl_pointcloud_raw.width = transformed_cloud.width;
  pcl_pointcloud_raw.height = transformed_cloud.height;
  pcl_pointcloud_raw.is_dense = transformed_cloud.is_dense == 1;
  pcl_pointcloud_raw.resize(transformed_cloud.width * transformed_cloud.height);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(transformed_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(transformed_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(transformed_cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> it_intensity(transformed_cloud, "intensity");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_intensity) {
    pcl::PointXYZI point;
    point.x = *it_x;
    point.y = *it_y;
    point.z = *it_z;
    point.intensity = static_cast<float>(*it_intensity);
    pcl_pointcloud_raw.emplace_back(std::move(point));
  }

  // generate feature map
  std::shared_ptr<FeatureMapInterface> feature_map_ptr =
    feature_generator_->generate(pcl_pointcloud_raw_ptr);

  CHECK_CUDA_ERROR(cudaMemcpy(
    input_d_.get(), feature_map_ptr->map_data.data(),
    feature_map_ptr->map_data.size() * sizeof(float), cudaMemcpyHostToDevice));

  std::vector<void *> buffers = {input_d_.get(), output_d_.get()};

  if (!trt_common_->setTensorsAddresses(buffers)) {
    return false;
  }
  trt_common_->enqueueV3(*stream_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output_h_.get(), output_d_.get(), sizeof(float) * output_size_, cudaMemcpyDeviceToHost,
    *stream_));
  cudaStreamSynchronize(*stream_);

  // post process
  const float objectness_thresh = 0.5;
  pcl::PointIndices valid_idx;
  valid_idx.indices.resize(pcl_pointcloud_raw_ptr->size());
  std::iota(valid_idx.indices.begin(), valid_idx.indices.end(), 0);
  cluster2d_->cluster(
    output_h_.get(), pcl_pointcloud_raw_ptr, valid_idx, objectness_thresh,
    true /*use all grids for clustering*/);
  const float height_thresh = 0.5;
  const int min_pts_num = 3;
  cluster2d_->getObjects(
    score_threshold_, height_thresh, min_pts_num, output, transformed_cloud.header);

  // move down pointcloud z_offset in z axis
  for (auto & feature_object : output.feature_objects) {
    sensor_msgs::msg::PointCloud2 updated_cloud;
    transformCloud(feature_object.feature.cluster, updated_cloud, -z_offset_);
    feature_object.feature.cluster = updated_cloud;
  }

  output.header = transformed_cloud.header;
  return true;
}
}  // namespace lidar_apollo_instance_segmentation
}  // namespace autoware
