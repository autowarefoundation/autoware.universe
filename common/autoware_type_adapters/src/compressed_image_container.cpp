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

#include "type_adapters/compressed_image_container.hpp"

#include "cuda.h"          // NOLINT
#include "cuda_runtime.h"  // NOLINT

#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"

#include <nvToolsExt.h>  // NOLINT

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware
{
namespace type_adaptation
{
namespace type_adapters
{

CompressedImageContainer::CompressedImageContainer()
{
  cuda_stream_ = std::make_shared<CUDAStreamWrapper>();
  cuda_event_ = std::make_shared<CUDAEventWrapper>();
}

CompressedImageContainer::CompressedImageContainer(
  std_msgs::msg::Header header, std::string format, size_t size_in_bytes,
  std::shared_ptr<CUDAStreamWrapper> cuda_stream)
: header_(header), cuda_stream_(cuda_stream), format_(format), size_in_bytes_(size_in_bytes)
{
  nvtxRangePushA("CompressedImageContainer:Create");
  cuda_mem_ = std::make_shared<CUDAMemoryWrapper>(this->size_in_bytes());
  cuda_event_ = std::make_shared<CUDAEventWrapper>();
  nvtxRangePop();
}

CompressedImageContainer::CompressedImageContainer(
  std::unique_ptr<sensor_msgs::msg::CompressedImage> unique_sensor_msgs_compressed_image)
: CompressedImageContainer(
    NotNull(
      unique_sensor_msgs_compressed_image.get(),
      "unique_sensor_msgs_compressed_image cannot be nullptr")
      .pointer->header,
    unique_sensor_msgs_compressed_image->format, unique_sensor_msgs_compressed_image->data.size())
{
  nvtxRangePushA("CompressedImage:CreateFromMessage");
  cuda_mem_->copy_to_device(
    &unique_sensor_msgs_compressed_image->data[0], size_in_bytes(), cuda_stream_->stream());
  nvtxRangePop();
}

CompressedImageContainer::CompressedImageContainer(
  const sensor_msgs::msg::CompressedImage & sensor_msgs_compressed_image)
: CompressedImageContainer(
    std::make_unique<sensor_msgs::msg::CompressedImage>(sensor_msgs_compressed_image))
{
}

CompressedImageContainer::CompressedImageContainer(const CompressedImageContainer & other)
{
  nvtxRangePushA("CompressedImageContainer:Copy");
  header_ = other.header_;
  format_ = other.format_;
  size_in_bytes_ = other.size_in_bytes_;

  // Make a new stream and have it wait on the previous one.
  cuda_stream_ = std::make_shared<CUDAStreamWrapper>();
  cuda_event_ = std::make_shared<CUDAEventWrapper>();
  cuda_event_->record(other.cuda_stream_);
  cudaStreamWaitEvent(cuda_stream_->stream(), cuda_event_->event());

  // Copy the image data over so that this is an independent copy (deep copy).
  cuda_mem_ = std::make_shared<CUDAMemoryWrapper>(size_in_bytes());
  if (
    cudaMemcpyAsync(
      other.cuda_mem_->device_memory(), cuda_mem_->device_memory(), size_in_bytes(),
      cudaMemcpyDeviceToDevice, cuda_stream_->stream()) != cudaSuccess) {
    throw std::runtime_error("Failed to copy memory from the GPU");
  }
  nvtxRangePop();
}

// Equals operator makes this container become another, shallow copy only.
CompressedImageContainer & CompressedImageContainer::operator=(
  const CompressedImageContainer & other)
{
  if (this == &other) {
    return *this;
  }

  header_ = other.header_;
  format_ = other.format_;
  size_in_bytes_ = other.size_in_bytes_;
  cuda_stream_ = other.cuda_stream_;
  cuda_event_ = other.cuda_event_;
  cuda_mem_ = other.cuda_mem_;

  return *this;
}

CompressedImageContainer::~CompressedImageContainer()
{
}

const std_msgs::msg::Header & CompressedImageContainer::header() const
{
  return header_;
}

std_msgs::msg::Header & CompressedImageContainer::header()
{
  return header_;
}

uint8_t * CompressedImageContainer::cuda_mem()
{
  return cuda_mem_->device_memory();
}

void CompressedImageContainer::get_sensor_msgs_compressed_image(
  sensor_msgs::msg::CompressedImage & destination) const
{
  nvtxRangePushA("CompressedImageContainer:GetMsg");
  destination.header = header_;
  destination.format = format_;
  destination.data.resize(size_in_bytes());
  cuda_mem_->copy_from_device(&destination.data[0], size_in_bytes(), cuda_stream_->stream());
  nvtxRangePop();
}

size_t CompressedImageContainer::size_in_bytes() const
{
  return size_in_bytes_;
}

}  //  namespace type_adapters
}  // namespace type_adaptation
}  //  namespace autoware
