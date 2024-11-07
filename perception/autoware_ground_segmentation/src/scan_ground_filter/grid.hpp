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

#ifndef SCAN_GROUND_FILTER__GRID_HPP_
#define SCAN_GROUND_FILTER__GRID_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/normalization.hpp>

#include <cmath>

namespace autoware::ground_segmentation
{

class ScanGroundGrid
{
public:
  ScanGroundGrid() = default;
  ~ScanGroundGrid() = default;

  void initialize(
    const float grid_size_m, const float grid_mode_switch_radius, const float virtual_lidar_z)
  {
    grid_size_m_ = grid_size_m;
    mode_switch_radius_ = grid_mode_switch_radius;
    virtual_lidar_z_ = virtual_lidar_z;

    // calculate parameters
    inv_grid_size_m_ = 1.0f / grid_size_m_;
    mode_switch_grid_id_ = mode_switch_radius_ * inv_grid_size_m_;
    mode_switch_angle_rad_ = std::atan2(mode_switch_radius_, virtual_lidar_z_);

    grid_size_rad_ = universe_utils::normalizeRadian(
                       std::atan2(mode_switch_radius_ + grid_size_m_, virtual_lidar_z_)) -
                     universe_utils::normalizeRadian(mode_switch_angle_rad_);
    inv_grid_size_rad_ = 1.0f / grid_size_rad_;
    tan_grid_size_rad_ = std::tan(grid_size_rad_);
    grid_id_offset_ = mode_switch_grid_id_ - mode_switch_angle_rad_ * inv_grid_size_rad_;

    is_initialized_ = true;
  }

  float getGridSize(const float radius, const size_t grid_id) const
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("ScanGroundGrid is not initialized.");
    }

    float grid_size = grid_size_m_;
    constexpr uint16_t back_steps_num = 1;

    if (radius > mode_switch_radius_ && grid_id > mode_switch_grid_id_ + back_steps_num) {
      // equivalent to grid_size = (std::tan(gamma) - std::tan(gamma - grid_size_rad_)) *
      // virtual_lidar_z_
      // where gamma = normalizeRadian(std::atan2(radius, virtual_lidar_z_), 0.0f)
      grid_size = radius - (radius - tan_grid_size_rad_ * virtual_lidar_z_) /
                             (1 + radius * tan_grid_size_rad_ / virtual_lidar_z_);
    }
    return grid_size;
  }

  uint16_t getGridId(const float radius) const
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("ScanGroundGrid is not initialized.");
    }

    uint16_t grid_id = 0;
    if (radius <= mode_switch_radius_) {
      grid_id = static_cast<uint16_t>(radius * inv_grid_size_m_);
    } else {
      auto gamma{universe_utils::normalizeRadian(std::atan2(radius, virtual_lidar_z_), 0.0f)};
      grid_id = grid_id_offset_ + gamma * inv_grid_size_rad_;
    }
    return grid_id;
  }

private:
  bool is_initialized_ = false;

  // configured parameters
  float grid_size_m_ = 0.0f;
  float mode_switch_radius_ = 0.0f;
  float virtual_lidar_z_ = 0.0f;

  // calculated parameters
  float inv_grid_size_m_ = 0.0f;
  float grid_size_rad_ = 0.0f;
  float inv_grid_size_rad_ = 0.0f;
  float tan_grid_size_rad_ = 0.0f;
  float mode_switch_grid_id_ = 0.0f;
  float mode_switch_angle_rad_ = 0.0f;
  float grid_id_offset_ = 0.0f;
};

}  // namespace autoware::ground_segmentation

#endif  // SCAN_GROUND_FILTER__GRID_HPP_
