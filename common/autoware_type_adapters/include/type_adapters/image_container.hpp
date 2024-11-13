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

#ifndef TYPE_ADAPTERS__IMAGE_CONTAINER_HPP_
#define TYPE_ADAPTERS__IMAGE_CONTAINER_HPP_

#include "cuda.h"          // NOLINT
#include "cuda_runtime.h"  // NOLINT
#include "rclcpp/type_adapter.hpp"
#include "type_adapters/cuda_stream_wrapper.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <memory>
#include <string>

namespace autoware
{
namespace type_adaptation
{
namespace type_adapters
{

/**
 * @brief Container class for managing images with CUDA memory
 *
 * Provides functionality to store and manage image data both in host and device memory,
 * with seamless conversion between ROS sensor_msgs::Image and CUDA memory
 */
class ImageContainer final
{
public:
  ImageContainer();

  /// @brief Constructor that takes ownership of a ROS image message
  explicit ImageContainer(std::unique_ptr<sensor_msgs::msg::Image> unique_sensor_msgs_image);

  /// @brief Constructor that copies a ROS image message
  explicit ImageContainer(const sensor_msgs::msg::Image & sensor_msgs_image);

  /// @brief Copy constructor
  ImageContainer(const ImageContainer & other);

  /// @brief Constructor with explicit image parameters
  ImageContainer(
    std_msgs::msg::Header header, uint32_t height, uint32_t width, std::string encoding,
    uint32_t step,
    std::shared_ptr<CUDAStreamWrapper> cuda_stream = std::make_shared<CUDAStreamWrapper>());

  ImageContainer & operator=(const ImageContainer & other);
  ~ImageContainer();

  /// @brief Get const reference to the ROS header
  const std_msgs::msg::Header & header() const;

  /// @brief Get mutable reference to the ROS header
  std_msgs::msg::Header & header();

  /// @brief Convert container data to ROS image message
  void get_sensor_msgs_image(sensor_msgs::msg::Image & destination) const;

  /// @brief Get pointer to CUDA device memory
  uint8_t * cuda_mem() const;

  /// @brief Get size of image data in bytes
  size_t size_in_bytes() const;

  /// @brief Get CUDA stream wrapper
  std::shared_ptr<CUDAStreamWrapper> cuda_stream() const { return cuda_stream_; }

  // Image property accessors
  uint32_t height() const { return height_; }
  uint32_t width() const { return width_; }
  const std::string & encoding() const { return encoding_; }
  uint32_t step() const { return step_; }

private:
  std_msgs::msg::Header header_;                    ///< ROS message header
  std::shared_ptr<CUDAStreamWrapper> cuda_stream_;  ///< CUDA stream for async operations
  std::shared_ptr<CUDAMemoryWrapper> cuda_mem_;     ///< CUDA memory manager
  std::shared_ptr<CUDAEventWrapper> cuda_event_;    ///< CUDA event manager

  uint32_t height_{0};    ///< Image height in pixels
  uint32_t width_{0};     ///< Image width in pixels
  std::string encoding_;  ///< Image encoding format
  uint32_t step_{0};      ///< Row step size in bytes
};

using ImageContainerUniquePtr = std::unique_ptr<ImageContainer>;
}  // namespace type_adapters
}  // namespace type_adaptation
}  // namespace autoware

/**
 * @brief ROS Type Adapter specialization for ImageContainer
 *
 * Enables automatic conversion between ROS sensor_msgs::Image and ImageContainer
 * for seamless integration with ROS publish/subscribe system
 */
template <>
struct rclcpp::TypeAdapter<
  autoware::type_adaptation::type_adapters::ImageContainer, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = autoware::type_adaptation::type_adapters::ImageContainer;
  using ros_message_type = sensor_msgs::msg::Image;

  /// @brief Convert ImageContainer to ROS Image message
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    source.get_sensor_msgs_image(destination);
  }

  /// @brief Convert ROS Image message to ImageContainer
  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination = autoware::type_adaptation::type_adapters::ImageContainer(source);
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  autoware::type_adaptation::type_adapters::ImageContainer, sensor_msgs::msg::Image);

#endif  // TYPE_ADAPTERS__IMAGE_CONTAINER_HPP_
