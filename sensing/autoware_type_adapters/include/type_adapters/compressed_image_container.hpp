// Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef TYPE_ADAPTERS__COMPRESSED_IMAGE_CONTAINER_HPP_
#define TYPE_ADAPTERS__COMPRESSED_IMAGE_CONTAINER_HPP_

#include "cuda.h"          // NOLINT
#include "cuda_runtime.h"  // NOLINT
#include "rclcpp/type_adapter.hpp"
#include "type_adapters/cuda_stream_wrapper.hpp"

#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"

#include <memory>
#include <string>

namespace autoware::type_adaptation::type_adapters
{

/**
 * @brief Container class for managing images with CUDA memory
 *
 * Provides functionality to store and manage image data both in host and device memory,
 * with seamless conversion between ROS sensor_msgs::CompressedImage and CUDA memory
 */
class CompressedImageContainer final
{
public:
  CompressedImageContainer();

  /// @brief Constructor that takes ownership of a ROS compressed image message
  explicit CompressedImageContainer(
    std::unique_ptr<sensor_msgs::msg::CompressedImage> unique_sensor_msgs_compressed_image);

  /// @brief Constructor that copies a ROS compressed image message
  explicit CompressedImageContainer(
    const sensor_msgs::msg::CompressedImage & sensor_msgs_compressed_image);

  /// @brief Copy constructor
  CompressedImageContainer(const CompressedImageContainer & other);

  /// @brief Constructor with explicit image parameters
  CompressedImageContainer(
    std_msgs::msg::Header header, std::string format, size_t size_in_bytes,
    std::shared_ptr<CUDAStreamWrapper> cuda_stream = std::make_shared<CUDAStreamWrapper>());

  CompressedImageContainer & operator=(const CompressedImageContainer & other);
  ~CompressedImageContainer();

  /// @brief Get const reference to the ROS header
  const std_msgs::msg::Header & header() const;

  /// @brief Get mutable reference to the ROS header
  std_msgs::msg::Header & header();

  /// @brief Convert container data to ROS compressed image message
  void get_sensor_msgs_compressed_image(sensor_msgs::msg::CompressedImage & destination) const;

  /// @brief Get pointer to CUDA device memory
  uint8_t * cuda_mem();

  /// @brief Get size of image data in bytes
  size_t size_in_bytes() const;

  /// @brief Get CUDA stream wrapper
  std::shared_ptr<CUDAStreamWrapper> cuda_stream() const { return cuda_stream_; }

  // Compressed Image property accessors
  const std::string & format() const { return format_; }

private:
  std_msgs::msg::Header header_;                    ///< ROS message header
  std::shared_ptr<CUDAStreamWrapper> cuda_stream_;  ///< CUDA stream for async operations
  std::shared_ptr<CUDAMemoryWrapper> cuda_mem_;     ///< CUDA memory manager
  std::shared_ptr<CUDAEventWrapper> cuda_event_;    ///< CUDA event manager

  std::string format_;  ///< Image encoding format
  uint32_t size_in_bytes_{0};
};

using CompressedImageContainerUniquePtr = std::unique_ptr<CompressedImageContainer>;

}  // namespace autoware::type_adaptation::type_adapters

/**
 * @brief ROS Type Adapter specialization for CompressedImageContainer
 *
 * Enables automatic conversion between ROS sensor_msgs::CompressedImage and
 * CompressedImageContainer for seamless integration with ROS publish/subscribe system
 */
template <>
struct rclcpp::TypeAdapter<
  autoware::type_adaptation::type_adapters::CompressedImageContainer,
  sensor_msgs::msg::CompressedImage>
{
  using is_specialized = std::true_type;
  using custom_type = autoware::type_adaptation::type_adapters::CompressedImageContainer;
  using ros_message_type = sensor_msgs::msg::CompressedImage;

  /// @brief Convert CompressedImageContainer to ROS Compressed Image message
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    source.get_sensor_msgs_compressed_image(destination);
  }

  /// @brief Convert ROS Compressed Image message to CompressedImageContainer
  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination = autoware::type_adaptation::type_adapters::CompressedImageContainer(source);
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  autoware::type_adaptation::type_adapters::CompressedImageContainer,
  sensor_msgs::msg::CompressedImage);

#endif  // TYPE_ADAPTERS__COMPRESSED_IMAGE_CONTAINER_HPP_
