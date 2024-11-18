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

namespace
{

float pseudoArcTan2(const float y, const float x)
{
  // lightweight arc tangent

  // avoid divide-by-zero
  if (y == 0.0f) {
    if (x >= 0.0f) return 0.0f;
    return M_PIf;
  }
  if (x == 0.0f) {
    if (y >= 0.0f) return M_PI_2f;
    return -M_PI_2f;
  }

  const float x_abs = std::abs(x);
  const float y_abs = std::abs(y);

  // divide to 8 zones
  if (x_abs > y_abs) {
    const float ratio = y_abs / x_abs;
    const float angle = ratio * M_PI_4f;
    if (y >= 0.0f) {
      if (x >= 0.0f) return angle;  // 1st zone
      return M_PIf - angle;         // 2nd zone
    } else {
      if (x >= 0.0f) return -angle;  // 4th zone
      return -M_PIf + angle;         // 3rd zone
    }
  } else {
    const float ratio = x_abs / y_abs;
    const float angle = ratio * M_PI_4f;
    if (y >= 0.0f) {
      if (x >= 0.0f) return M_PI_2f - angle;  // 1st zone
      return M_PI_2f + angle;                 // 2nd zone
    } else {
      if (x >= 0.0f) return -M_PI_2f - angle;  // 4th zone
      return -M_PI_2f + angle;                 // 3rd zone
    }
  }
}
}  // namespace
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
    grid_dist_size_ = 1.0f;
    grid_azimuth_size_ = 0.1f;
    grid_linearity_switch_radius_ = 20.0f;
    grid_radial_limit_ = 200.0f;  // meters
    grid_dist_size_rad_ = 0.0f;
    grid_linearity_switch_num_ = 0;
    grid_linearity_switch_angle_ = 0.0f;
    grid_size_rad_inv_ = 0.0f;
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

    // set grid linearity switch radius
    grid_linearity_switch_num_ = static_cast<int>(grid_linearity_switch_radius / grid_dist_size_);
    grid_linearity_switch_radius_ = grid_linearity_switch_num_ * grid_dist_size_;

    // calculate grid parameters
    grid_dist_size_rad_ =
      pseudoArcTan2(grid_linearity_switch_radius_ + grid_dist_size_, origin_z_) -
      pseudoArcTan2(grid_linearity_switch_radius_, origin_z_);

    // generate grid geometry
    setGridBoundaries();

    // initialize and resize cells
    cells_.clear();
    cells_.resize(radial_idx_offsets_.back() + azimuth_grids_per_radial_.back());

    // set cell geometry
    setCellGeometry();

    // set initialized flag
    is_initialized_ = true;
  }

  // method to add a point to the grid
  void addPoint(const float x, const float y, const size_t point_idx)
  {
    // calculate the grid id
    const int grid_idx = getGridIdx(x, y);

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

      // find and link the scan-grid root cell
      {
        bool is_prev_found = false;
        int scan_grid_root_idx = cell.prev_grid_idx_;
        while (scan_grid_root_idx >= 0) {
          auto & prev_cell = cells_[scan_grid_root_idx];
          if (prev_cell.isEmpty()) {
            // check previous of the previous cell
            scan_grid_root_idx = prev_cell.prev_grid_idx_;
          } else {
            // not empty, then set the previous cell
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
  float grid_radial_limit_;            // meters
  float grid_dist_size_rad_;           // radians
  int grid_linearity_switch_num_;      // number of grids within the switch radius
  float grid_linearity_switch_angle_;  // angle at the switch radius
  float grid_size_rad_inv_;            // inverse of the grid size in radians
  bool is_initialized_ = false;

  // array of grid boundaries
  std::vector<float> grid_radial_boundaries_;
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
      // constant distance
      for (int i = 0; i < grid_linearity_switch_num_; i++) {
        grid_radial_boundaries_.push_back(i * grid_dist_size_);
      }
      // at the switch radius
      grid_radial_boundaries_.push_back(grid_linearity_switch_radius_);
      // constant angle
      grid_linearity_switch_angle_ = pseudoArcTan2(grid_linearity_switch_radius_, origin_z_);
      float angle = grid_linearity_switch_angle_;
      const float grid_angle_interval =
        pseudoArcTan2(grid_linearity_switch_radius_ + grid_dist_size_, origin_z_) - angle;
      grid_size_rad_inv_ = 1.0f / grid_angle_interval;
      angle += grid_angle_interval;
      while (angle < M_PI_2) {
        const float dist = tan(angle) * origin_z_;
        grid_radial_boundaries_.push_back(dist);
        if (dist > grid_radial_limit_) {
          break;
        }
        angle += grid_angle_interval;
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

      constexpr float dist_to_saturate = 20.0f;

      for (size_t i = 1; i < radial_grid_num; ++i) {
        const int max_azimuth_grid_num = static_cast<int>(2.0 * M_PI / grid_azimuth_size_);
        const float dist = grid_radial_boundaries_[i];
        const float decay = std::min(dist / dist_to_saturate, 1.0f);
        const int grid_num = static_cast<int>(max_azimuth_grid_num * decay);

        const int azimuth_grid_num = std::max(std::min(grid_num, max_azimuth_grid_num), 1);
        const float azimuth_interval_evened = 2.0f * M_PI / azimuth_grid_num;
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

    // constant distance
    if (radius < grid_linearity_switch_radius_) {
      grid_rad_idx = static_cast<int>(radius / grid_dist_size_);
    } else if (radius < grid_radial_limit_) {
      const float angle = pseudoArcTan2(radius, origin_z_);
      grid_rad_idx = grid_linearity_switch_num_ +
                     static_cast<int>((angle - grid_linearity_switch_angle_) * grid_size_rad_inv_);
    }

    // check if the grid id is valid
    if (grid_rad_idx >= static_cast<int>(grid_radial_boundaries_.size())) {
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
  int getGridIdx(const float & x, const float & y) const
  {
    const float x_fixed = x - origin_x_;
    const float y_fixed = y - origin_y_;

    const float radius = std::sqrt(x_fixed * x_fixed + y_fixed * y_fixed);
    const float azimuth = pseudoArcTan2(y_fixed, x_fixed);

    const int grid_rad_idx = getRadialIdx(radius);
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
      cell.scan_grid_root_idx_ = -1;
    }
  }
};

}  // namespace autoware::ground_segmentation

#endif  // SCAN_GROUND_FILTER__GRID_HPP_
