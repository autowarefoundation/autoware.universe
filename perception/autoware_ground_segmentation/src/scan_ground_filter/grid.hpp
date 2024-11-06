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

#include <glob.h>

#include <algorithm>
#include <cmath>
#include <vector>

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
    mode_switch_grid_idx_ = mode_switch_radius_ * inv_grid_size_m_;
    mode_switch_angle_rad_ = std::atan2(mode_switch_radius_, virtual_lidar_z_);

    grid_size_rad_ = universe_utils::normalizeRadian(
                       std::atan2(mode_switch_radius_ + grid_size_m_, virtual_lidar_z_)) -
                     universe_utils::normalizeRadian(mode_switch_angle_rad_);
    inv_grid_size_rad_ = 1.0f / grid_size_rad_;
    tan_grid_size_rad_ = std::tan(grid_size_rad_);
    grid_idx_offset_ = mode_switch_grid_idx_ - mode_switch_angle_rad_ * inv_grid_size_rad_;

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

    if (radius > mode_switch_radius_ && grid_id > mode_switch_grid_idx_ + back_steps_num) {
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
      grid_id = grid_idx_offset_ + gamma * inv_grid_size_rad_;
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
  float mode_switch_grid_idx_ = 0.0f;
  float mode_switch_angle_rad_ = 0.0f;
  float grid_idx_offset_ = 0.0f;
};

class Cell
{
public:
  // list of point indices
  std::vector<size_t> point_indices_;

  // method to check if the cell is empty
  bool isEmpty() const { return point_indices_.empty(); }
  int getPointNum() const { return point_indices_.size(); }

  // index of the cell
  int grid_idx_;
  int radial_idx_;
  int azimuth_idx_;
  int next_grid_idx_;
  int prev_grid_idx_;

  // geometric properties of the cell
  float center_radius_;
  float center_azimuth_;
  float radial_size_;
  float azimuth_size_;

  // statistics of the points in the cell
  float avg_height_;
  float max_height_;
  float min_height_;
  float std_dev_height_;

  // process flags
  bool is_processed_;
};

class Grid
{
public:
  Grid(const float origin_x, const float origin_y, const float origin_z)
  : origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z)
  {
  }
  ~Grid() = default;

  void initialize(
    const float grid_dist_size, const float grid_azimuth_size,
    const float grid_linearity_switch_radius)
  {
    grid_dist_size_ = grid_dist_size;
    grid_azimuth_size_ = grid_azimuth_size;
    grid_linearity_switch_radius_ = grid_linearity_switch_radius;

    // calculate grid parameters
    grid_radial_limit_ = 160.0f;  // [m]
    grid_dist_size_rad_ = std::atan2(grid_linearity_switch_radius_ + grid_dist_size_, origin_z_) -
                          std::atan2(grid_linearity_switch_radius_, origin_z_);

    // generate grid geometry
    setGridBoundaries();

    // initialize and resize cells
    cells_.clear();
    cells_.resize(radial_idx_offsets_.back() + azimuth_grids_per_radial_.back());

    // set cell geometry
    setCellGeometry();

    // set initialized flag
    is_initialized_ = true;

    {
      // print debug information, size,
      std::cout << "Grid initialized." << std::endl;
      // grid_radial_limit_
      std::cout << "Grid radial limit: " << grid_radial_limit_ << std::endl;
      // distance grid size and positions
      std::cout << "Grid distance size: " << grid_dist_size_ << std::endl;
      for (size_t i = 0; i < grid_radial_boundaries_.size(); ++i) {
        std::cout << "Grid radial boundary: " << grid_radial_boundaries_[i] << std::endl;
      }
      std::cout << "Grid radial size: " << grid_radial_boundaries_.size() << std::endl;
      // azimuth grid size
      std::cout << "Grid azimuth interval: " << grid_azimuth_size_ * 180 / (M_PI) << std::endl;
      for (size_t i = 0; i < azimuth_grids_per_radial_.size(); ++i) {
        std::cout << "Grid azimuth number: " << azimuth_grids_per_radial_[i]
                  << ", Grid azimuth interval: " << azimuth_interval_per_radial_[i] * 180 / (M_PI)
                  << std::endl;
      }
      // offset list
      for (size_t i = 0; i < radial_idx_offsets_.size(); ++i) {
        std::cout << "Grid id offset: " << radial_idx_offsets_[i] << std::endl;
      }
      std::cout << "Grid size: " << cells_.size() << std::endl;
    }
  }

  // method to add a point to the grid
  void addPoint(const float x, const float y, const size_t point_idx)
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("Grid is not initialized.");
    }

    // calculate the grid id
    const float radius = std::hypot(x, y);
    const float azimuth = std::atan2(y, x);
    const int grid_id = getGridIdx(radius, azimuth);

    // check if the point is within the grid
    if (grid_id < 0) {
      return;
    }

    // add the point to the cell
    cells_[grid_id].point_indices_.push_back(point_idx);
  }

  size_t getGridSize() const
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("Grid is not initialized.");
    }

    return cells_.size();
  }

  // method to get the cell
  const Cell & getCell(const int grid_id) const
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("Grid is not initialized.");
    }

    return cells_[grid_id];
  }

  void resetCells()
  {
    for (auto & cell : cells_) {
      cell.point_indices_.clear();
    }
  }

  void setGridStatistics()
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("Grid is not initialized.");
    }

    // debug information for new grid

    // check number of points in cells, azimuth grid index of 0
    // heading line of cells
    for (size_t i = 0; i < radial_idx_offsets_.size(); ++i) {
      const int radial_idx = i;
      const int azimuth_idx = 3;
      const int cell_idx = radial_idx_offsets_[radial_idx] + azimuth_idx;
      const Cell & cell = cells_[cell_idx];
      std::cout << "====== Grid id: " << cell_idx << ", Number of points: " << cell.getPointNum()
                << std::endl;

      // print index of the cell
      std::cout << "index: " << cell.grid_idx_ << " radial: " << cell.radial_idx_
                << " azimuth: " << cell.azimuth_idx_ << std::endl;

      // print position of the cell
      std::cout << "position radius: " << cell.center_radius_
                << " azimuth: " << cell.center_azimuth_ * 180 / M_PI << std::endl;

      // print next grid, only exists
      if (cell.next_grid_idx_ >= 0) {
        const Cell & next_cell = cells_[cell.next_grid_idx_];
        std::cout << "--- next grid id: " << next_cell.grid_idx_
                  << ", position radius: " << next_cell.center_radius_
                  << " azimuth: " << next_cell.center_azimuth_ * 180 / M_PI << std::endl;
      }

      // print previous grid, only exists
      if (cell.prev_grid_idx_ >= 0) {
        const Cell & prev_cell = cells_[cell.prev_grid_idx_];
        std::cout << "--- prev grid id: " << prev_cell.grid_idx_
                  << ", position radius: " << prev_cell.center_radius_
                  << " azimuth: " << prev_cell.center_azimuth_ * 180 / M_PI << std::endl;
      }
    }
  }

private:
  // given parameters
  float origin_x_;
  float origin_y_;
  float origin_z_;
  float grid_dist_size_;                // meters
  float grid_azimuth_size_;             // radians
  float grid_linearity_switch_radius_;  // meters

  // calculated parameters
  float grid_radial_limit_;   // meters
  float grid_dist_size_rad_;  // radians
  bool is_initialized_ = false;

  // array of grid boundaries
  std::vector<float> grid_radial_boundaries_;
  std::vector<int> azimuth_grids_per_radial_;
  std::vector<float> azimuth_interval_per_radial_;
  std::vector<int> radial_idx_offsets_;

  // Generate grid geometry
  // the grid is cylindrical mesh grid
  // azimuth interval: constant angle
  // radial interval: constant distance within mode switch radius
  //                  constant elevation angle outside mode switch radius
  void setGridBoundaries()
  {
    // radial boundaries
    {
      int idx = 0;
      float radius_initial = 1.0f;    // initial grid of 1 meter
      float radius = radius_initial;  // initial grid of 1 meter
      // 1. within mode switch radius, constant distance
      while (radius < grid_linearity_switch_radius_) {
        grid_radial_boundaries_.push_back(radius);
        idx++;
        radius = static_cast<float>(idx) * grid_dist_size_ + radius_initial;
      }
      // 2. outside mode switch radius, constant elevation angle
      while (radius < grid_radial_limit_ && radius > 0) {
        grid_radial_boundaries_.push_back(radius);
        const float angle = std::atan2(radius, origin_z_);
        radius = tan(angle + grid_dist_size_rad_) * origin_z_;
      }
    }

    // azimuth boundaries
    {
      if (grid_azimuth_size_ <= 0) {
        throw std::runtime_error("Grid azimuth size is not positive.");
      }

      // number of azimuth grids per radial grid
      const int azimuth_grid_num = std::max(static_cast<int>(2.0 * M_PI / grid_azimuth_size_), 1);
      const float azimuth_interval_evened = 2.0f * M_PI / azimuth_grid_num;
      azimuth_grids_per_radial_.resize(grid_radial_boundaries_.size());
      azimuth_interval_per_radial_.resize(grid_radial_boundaries_.size());
      for (size_t i = 0; i < grid_radial_boundaries_.size(); ++i) {
        // constant azimuth interval
        azimuth_grids_per_radial_[i] = azimuth_grid_num;
        azimuth_interval_per_radial_[i] = azimuth_interval_evened;
      }
    }

    // accumulate the number of azimuth grids per radial grid, set offset for each radial grid
    radial_idx_offsets_.resize(grid_radial_boundaries_.size());
    radial_idx_offsets_[0] = 0;
    for (size_t i = 1; i < grid_radial_boundaries_.size(); ++i) {
      radial_idx_offsets_[i] = radial_idx_offsets_[i - 1] + azimuth_grids_per_radial_[i - 1];
    }
  }

  int getAzimuthGridIdx(const int & radial_idx, const float & azimuth) const
  {
    // constant azimuth interval
    return static_cast<int>(azimuth / azimuth_interval_per_radial_[radial_idx]);
  }

  int getRadialIdx(const float & radius) const
  {
    // check if the point is within the grid
    if (radius > grid_radial_limit_) {
      return -1;
    }
    if (radius < 0) {
      return -1;
    }

    // determine the grid id
    int grid_rad_idx = -1;
    // radial grid id
    for (size_t i = 0; i < grid_radial_boundaries_.size(); ++i) {
      if (radius < grid_radial_boundaries_[i]) {
        grid_rad_idx = i;
        break;
      }
    }

    return grid_rad_idx;
  }

  int getGridIdx(const int & radial_idx, const int & azimuth_idx) const
  {
    return radial_idx_offsets_[radial_idx] + azimuth_idx;
  }

  // method to determine the grid id of a point
  // -1 means out of range
  // range limit is horizon angle
  int getGridIdx(const float radius, const float azimuth) const
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("Grid is not initialized.");
    }

    int grid_rad_idx = getRadialIdx(radius);
    if (grid_rad_idx < 0) {
      return -1;
    }

    // normalize azimuth, make sure it is within [0, 2pi)
    float azimuth_norm = universe_utils::normalizeRadian(azimuth, 0.0f);

    // azimuth grid id
    int grid_az_idx = getAzimuthGridIdx(grid_rad_idx, azimuth_norm);
    if (grid_az_idx < 0) {
      return -1;
    }

    return getGridIdx(grid_rad_idx, grid_az_idx);
  }

  void getRadialAzimuthIdxFromCellIdx(const int cell_id, int & radial_idx, int & azimuth_idx) const
  {
    radial_idx = -1;
    azimuth_idx = -1;
    for (size_t i = 0; i < radial_idx_offsets_.size(); ++i) {
      if (cell_id < radial_idx_offsets_[i]) {
        radial_idx = i - 1;
        azimuth_idx = cell_id - radial_idx_offsets_[i - 1];
        break;
      }
    }
    if (cell_id >= radial_idx_offsets_.back()) {
      radial_idx = radial_idx_offsets_.size() - 1;
      azimuth_idx = cell_id - radial_idx_offsets_.back();
    }
  }

  void setCellGeometry()
  {
    for (size_t idx = 0; idx < cells_.size(); ++idx) {
      Cell & cell = cells_[idx];

      int radial_idx = 0;
      int azimuth_idx = 0;
      getRadialAzimuthIdxFromCellIdx(idx, radial_idx, azimuth_idx);

      cell.grid_idx_ = idx;
      cell.radial_idx_ = radial_idx;
      cell.azimuth_idx_ = azimuth_idx;

      // set width of the cell
      const auto radial_grid_num = static_cast<int>(grid_radial_boundaries_.size() - 1);
      if (radial_idx < radial_grid_num) {
        cell.radial_size_ =
          grid_radial_boundaries_[radial_idx + 1] - grid_radial_boundaries_[radial_idx];
      } else {
        cell.radial_size_ = grid_radial_limit_ - grid_radial_boundaries_[radial_idx];
      }
      cell.azimuth_size_ = azimuth_interval_per_radial_[radial_idx];

      // set center of the cell
      cell.center_radius_ = grid_radial_boundaries_[radial_idx] + cell.radial_size_ * 0.5f;
      cell.center_azimuth_ = (static_cast<float>(azimuth_idx) + 0.5f) * cell.azimuth_size_;

      // set next grid id, which is radially next
      int next_grid_id = -1;
      // only if the next radial grid exists
      if (radial_idx < radial_grid_num) {
        // find nearest azimuth grid in the next radial grid
        const float azimuth = cell.center_azimuth_;
        const size_t azimuth_idx_next_radial_grid = getAzimuthGridIdx(radial_idx + 1, azimuth);
        next_grid_id = radial_idx_offsets_[radial_idx + 1] + azimuth_idx_next_radial_grid;
        next_grid_id = getGridIdx(radial_idx + 1, azimuth_idx_next_radial_grid);
      }
      cell.next_grid_idx_ = next_grid_id;

      // set previous grid id, which is radially previous
      int prev_grid_id = -1;
      // only if the previous radial grid exists
      if (radial_idx > 0) {
        // find nearest azimuth grid in the previous radial grid
        const float azimuth = cell.center_azimuth_;
        // constant azimuth interval
        const size_t azimuth_idx_prev_radial_grid = getAzimuthGridIdx(radial_idx - 1, azimuth);
        prev_grid_id = getGridIdx(radial_idx - 1, azimuth_idx_prev_radial_grid);
      }
      cell.prev_grid_idx_ = prev_grid_id;
    }
  }

  // list of cells
  std::vector<Cell> cells_;
};

}  // namespace autoware::ground_segmentation

#endif  // SCAN_GROUND_FILTER__GRID_HPP_
