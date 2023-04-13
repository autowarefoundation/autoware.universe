// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <Eigen/Core>
#include <pcdless_common/ground_plane.hpp>
#include <rclcpp/time.hpp>

#include <optional>

namespace pcdless::ground_server
{
class LowPassFilter
{
public:
  using Ground = common::GroundPlane;

  LowPassFilter() { initialize(0); }

  void initialize(const float & height)
  {
    height_ = height;
    cov_height_ = 25.f;

    initialized = true;
  }

  void update(const float height)
  {
    cov_height_ += 0.01;

    constexpr float noise_height = 0.2f;
    float error = height - height_;
    float gain = cov_height_ / (cov_height_ + noise_height);

    height_ += gain * error;
    cov_height_ = cov_height_ - gain * cov_height_;
  }

  float get_estimate() const { return height_; }

private:
  bool initialized = false;

  float height_;
  float cov_height_;
};
}  // namespace pcdless::ground_server