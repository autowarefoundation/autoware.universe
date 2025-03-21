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

#include "autoware/lidar_bevfusion/preprocess/pointcloud_densification.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/optional.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <utility>

namespace
{

boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
    return transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lidar_bevfusion"), ex.what());
    return boost::none;
  }
}

Eigen::Affine3f transformToEigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Affine3f a;
  a.matrix() = tf2::transformToEigen(t).matrix().cast<float>();
  return a;
}

}  // namespace

namespace autoware::lidar_bevfusion
{

PointCloudDensification::PointCloudDensification(
  const DensificationParam & param, cudaStream_t & stream)
: param_(param), stream_(stream)
{
}

bool PointCloudDensification::enqueuePointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg, const tf2_ros::Buffer & tf_buffer)
{
  const auto header = pointcloud_msg.header;

  if (param_.pointcloud_cache_size() > 1) {
    auto transform_world2current =
      getTransform(tf_buffer, header.frame_id, param_.world_frame_id(), header.stamp);
    if (!transform_world2current) {
      return false;
    }
    auto affine_world2current = transformToEigen(transform_world2current.get());

    enqueue(pointcloud_msg, affine_world2current);
  } else {
    enqueue(pointcloud_msg, Eigen::Affine3f::Identity());
  }

  dequeue();

  return true;
}

void PointCloudDensification::enqueue(
  const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Affine3f & affine_world2current)
{
  affine_world2current_ = affine_world2current;
  current_timestamp_ = rclcpp::Time(msg.header.stamp).seconds();

  auto data_d = autoware::cuda_utils::make_unique<InputPointType[]>(msg.width * msg.height);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    data_d.get(), msg.data.data(), sizeof(std::uint8_t) * msg.width * msg.height * msg.point_step,
    cudaMemcpyHostToDevice, stream_));

  PointCloudWithTransform pointcloud = {
    std::move(data_d), msg.header, msg.width * msg.height, affine_world2current.inverse()};

  pointcloud_cache_.push_front(std::move(pointcloud));
}

void PointCloudDensification::dequeue()
{
  if (pointcloud_cache_.size() > param_.pointcloud_cache_size()) {
    pointcloud_cache_.pop_back();
  }
}

}  // namespace autoware::lidar_bevfusion
