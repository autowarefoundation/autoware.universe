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
#include <autoware/universe_utils/system/time_keeper.hpp>

#include <glob.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::ground_segmentation
{
using autoware::universe_utils::ScopedTimeTrack;

// Concentric Zone Model (CZM) based polar grid
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

  int scan_grid_root_idx_;

  // geometric properties of the cell
  float center_radius_;
  float center_azimuth_;
  float radial_size_;
  float azimuth_size_;

  // ground statistics of the points in the cell
  float avg_height_;
  float max_height_;
  float min_height_;
  float avg_radius_;
  float gradient_;
  float intercept_;

  // process flags
  bool is_processed_ = false;
  bool is_ground_initialized_ = false;
  bool has_ground_ = false;
};

class Grid
{
public:
  Grid(const float origin_x, const float origin_y, const float origin_z)
  : origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z)
  {
  }
  ~Grid() = default;

  void setTimeKeeper(std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_ptr)
  {
    time_keeper_ = std::move(time_keeper_ptr);
  }

  void initialize(
    const float grid_dist_size, const float grid_azimuth_size,
    const float grid_linearity_switch_radius)
  {
    grid_dist_size_ = grid_dist_size;
    grid_azimuth_size_ = grid_azimuth_size;
    grid_linearity_switch_radius_ = grid_linearity_switch_radius;

    // calculate grid parameters
    grid_radial_limit_ = 160.0f;  // [m]
    grid_radial_limit_sq_ = grid_radial_limit_ * grid_radial_limit_;
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
    // calculate the grid id
    const float radius_sq = x * x + y * y;
    const float azimuth = std::atan2(y, x);
    const int grid_idx = getGridIdx(radius_sq, azimuth);

    // check if the point is within the grid
    if (grid_idx < 0) {
      return;
    }
    const size_t grid_idx_idx = static_cast<size_t>(grid_idx);

    // check cell index is valid
    if (grid_idx_idx >= cells_.size()) {
      throw std::runtime_error("Invalid grid id when trying to add a point.");
    }

    // add the point to the cell
    cells_[grid_idx_idx].point_indices_.emplace_back(point_idx);
  }

  size_t getGridSize() const { return cells_.size(); }

  // method to get the cell
  Cell & getCell(const int grid_idx)
  {
    const size_t idx = static_cast<size_t>(grid_idx);
    if (grid_idx < 0 || idx >= cells_.size()) {
      throw std::runtime_error("Invalid grid cell index to get.");
    }

    return cells_[idx];
  }

  void resetCells()
  {
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

    for (auto & cell : cells_) {
      cell.point_indices_.clear();
      cell.is_processed_ = false;
      cell.is_ground_initialized_ = false;
      cell.scan_grid_root_idx_ = -1;
      cell.has_ground_ = false;
      cell.avg_height_ = 0.0f;
      cell.max_height_ = 0.0f;
      cell.min_height_ = 0.0f;
      cell.avg_radius_ = 0.0f;
      cell.gradient_ = 0.0f;
      cell.intercept_ = 0.0f;
    }
  }

  // may not needed
  void setGridStatistics()
  {
    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("Grid is not initialized.");
    }

    // debug information for new grid

    // check a line of cells
    int current_grid_idx = 3;
    while (current_grid_idx >= 0) {
      const Cell & cell = cells_[current_grid_idx];
      std::cout << "====== Grid id: " << cell.grid_idx_
                << ", Number of points: " << cell.getPointNum() << std::endl;

      // print previous grid, only exists
      if (cell.prev_grid_idx_ >= 0) {
        const Cell & prev_cell = cells_[cell.prev_grid_idx_];
        std::cout << "- prev grid id: " << prev_cell.grid_idx_
                  << ", position radius: " << prev_cell.center_radius_
                  << " azimuth: " << prev_cell.center_azimuth_ * 180 / M_PI << std::endl;
      }

      // print index of the cell
      std::cout << "- curr grid id: " << cell.grid_idx_
                << ", position radius: " << cell.center_radius_
                << " azimuth: " << cell.center_azimuth_ * 180 / M_PI << std::endl;

      // print position of the cell
      std::cout << "index radial: " << cell.radial_idx_ << " azimuth: " << cell.azimuth_idx_
                << std::endl;

      // print next grid, only exists
      if (cell.next_grid_idx_ >= 0) {
        const Cell & next_cell = cells_[cell.next_grid_idx_];
        std::cout << "- next grid id: " << next_cell.grid_idx_
                  << ", position radius: " << next_cell.center_radius_
                  << " azimuth: " << next_cell.center_azimuth_ * 180 / M_PI << std::endl;
      }
      current_grid_idx = cell.next_grid_idx_;
    }
  }

  void setGridConnections()
  {
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

    // check if initialized
    if (!is_initialized_) {
      throw std::runtime_error("Grid is not initialized.");
    }

    // iterate over grid cells
    for (size_t i = 0; i < cells_.size(); ++i) {
      Cell & cell = cells_[i];

      // check if the cell is empty
      if (cell.isEmpty()) {
        continue;
      }

      // check the next cell
      {
        bool is_next_found = false;
        int next_cell_idx = cell.next_grid_idx_;
        while (next_cell_idx >= 0) {
          auto & next_cell = cells_[next_cell_idx];
          if (next_cell.isEmpty()) {
            // check next of the next cell
            next_cell_idx = next_cell.next_grid_idx_;
          } else {
            // not empty, set the next cell
            cell.next_grid_idx_ = next_cell_idx;
            is_next_found = true;
            break;
          }
        }
        if (!is_next_found) {
          cell.next_grid_idx_ = -1;
        }
      }

      // check the previous cell
      {
        bool is_prev_found = false;
        int scan_grid_root_idx = cell.prev_grid_idx_;
        while (scan_grid_root_idx >= 0) {
          auto & prev_cell = cells_[scan_grid_root_idx];
          if (prev_cell.isEmpty()) {
            // check previous of the previous cell
            scan_grid_root_idx = prev_cell.prev_grid_idx_;
          } else {
            // not empty, set the previous cell
            cell.scan_grid_root_idx_ = scan_grid_root_idx;
            is_prev_found = true;
            break;
          }
        }
        if (!is_prev_found) {
          cell.scan_grid_root_idx_ = -1;
        }
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
  float grid_radial_limit_;     // meters
  float grid_radial_limit_sq_;  // meters
  float grid_dist_size_rad_;    // radians
  bool is_initialized_ = false;

  // array of grid boundaries
  std::vector<float> grid_radial_boundaries_;
  std::vector<float> grid_radial_boundaries_sq_;
  std::vector<int> azimuth_grids_per_radial_;
  std::vector<float> azimuth_interval_per_radial_;
  std::vector<int> radial_idx_offsets_;

  // list of cells
  std::vector<Cell> cells_;

  // debug information
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;

  // Generate grid geometry
  // the grid is cylindrical mesh grid
  // azimuth interval: constant angle
  // radial interval: constant distance within mode switch radius
  //                  constant elevation angle outside mode switch radius
  void setGridBoundaries()
  {
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

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

      // square of the boundaries
      grid_radial_boundaries_sq_.resize(grid_radial_boundaries_.size());
      for (size_t i = 0; i < grid_radial_boundaries_.size(); ++i) {
        grid_radial_boundaries_sq_[i] = grid_radial_boundaries_[i] * grid_radial_boundaries_[i];
      }
    }
    const size_t radial_grid_num = grid_radial_boundaries_.size();

    // azimuth boundaries
    {
      if (grid_azimuth_size_ <= 0) {
        throw std::runtime_error("Grid azimuth size is not positive.");
      }

      // number of azimuth grids per radial grid
      azimuth_grids_per_radial_.resize(radial_grid_num);
      azimuth_interval_per_radial_.resize(radial_grid_num);
      azimuth_grids_per_radial_[0] = 1;
      azimuth_interval_per_radial_[0] = 2.0f * M_PI;
      const int azimuth_grid_num = std::max(static_cast<int>(2.0 * M_PI / grid_azimuth_size_), 1);
      const float azimuth_interval_evened = 2.0f * M_PI / azimuth_grid_num;
      for (size_t i = 1; i < radial_grid_num; ++i) {
        // constant azimuth interval
        azimuth_grids_per_radial_[i] = azimuth_grid_num;
        azimuth_interval_per_radial_[i] = azimuth_interval_evened;
      }
    }

    // accumulate the number of azimuth grids per radial grid, set offset for each radial grid
    radial_idx_offsets_.resize(radial_grid_num);
    radial_idx_offsets_[0] = 0;
    for (size_t i = 1; i < radial_grid_num; ++i) {
      radial_idx_offsets_[i] = radial_idx_offsets_[i - 1] + azimuth_grids_per_radial_[i - 1];
    }
  }

  int getAzimuthGridIdx(const int & radial_idx, const float & azimuth) const
  {
    const int azimuth_grid_num = azimuth_grids_per_radial_[radial_idx];

    int azimuth_grid_idx =
      static_cast<int>(std::floor(azimuth / azimuth_interval_per_radial_[radial_idx]));
    if (azimuth_grid_idx == azimuth_grid_num) {
      // loop back to the first grid
      azimuth_grid_idx = 0;
    }
    // constant azimuth interval
    return azimuth_grid_idx;
  }

  int getRadialIdx(const float & radius_sq) const
  {
    // check if the point is within the grid
    if (radius_sq > grid_radial_limit_sq_) {
      return -1;
    }
    if (radius_sq < 0) {
      return -1;
    }

    // determine the grid id
    int grid_rad_idx = -1;

    // speculate the grid id from the radius, underestimate it to do not miss the grid
    size_t grid_idx_speculated = 0;
    float radius = std::sqrt(radius_sq);
    if (radius < grid_linearity_switch_radius_) {
      grid_idx_speculated = static_cast<size_t>(std::sqrt(radius_sq) / grid_dist_size_);
    } else {
      grid_idx_speculated = static_cast<size_t>(grid_linearity_switch_radius_);
    }

    // radial grid id
    for (size_t i = grid_idx_speculated; i < grid_radial_boundaries_sq_.size(); ++i) {
      if (radius_sq < grid_radial_boundaries_sq_[i]) {
        grid_rad_idx = i;
        break;
      }
    }

    // check if the grid id is valid
    if (grid_rad_idx >= static_cast<int>(grid_radial_boundaries_sq_.size())) {
      // throw error
      throw std::runtime_error("Invalid radial grid index.");
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
  int getGridIdx(const float & radius_sq, const float & azimuth) const
  {
    const int grid_rad_idx = getRadialIdx(radius_sq);
    if (grid_rad_idx < 0) {
      return -1;
    }

    // normalize azimuth, make sure it is within [0, 2pi)
    const float azimuth_norm = universe_utils::normalizeRadian(azimuth, 0.0f);

    // azimuth grid id
    const int grid_az_idx = getAzimuthGridIdx(grid_rad_idx, azimuth_norm);
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
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

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
      int next_grid_idx = -1;
      // only if the next radial grid exists
      if (radial_idx < radial_grid_num) {
        // find nearest azimuth grid in the next radial grid
        const float azimuth = cell.center_azimuth_;
        const size_t azimuth_idx_next_radial_grid = getAzimuthGridIdx(radial_idx + 1, azimuth);
        next_grid_idx = getGridIdx(radial_idx + 1, azimuth_idx_next_radial_grid);
      }
      cell.next_grid_idx_ = next_grid_idx;

      // set previous grid id, which is radially previous
      int prev_grid_idx = -1;
      // only if the previous radial grid exists
      if (radial_idx > 0) {
        // find nearest azimuth grid in the previous radial grid
        const float azimuth = cell.center_azimuth_;
        // constant azimuth interval
        const size_t azimuth_idx_prev_radial_grid = getAzimuthGridIdx(radial_idx - 1, azimuth);
        prev_grid_idx = getGridIdx(radial_idx - 1, azimuth_idx_prev_radial_grid);
      }
      cell.prev_grid_idx_ = prev_grid_idx;
      cell.scan_grid_root_idx_ = prev_grid_idx;
    }
  }
};

}  // namespace autoware::ground_segmentation

#endif  // SCAN_GROUND_FILTER__GRID_HPP_
