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

#include "grid_ground_filter.hpp"

#include "data.hpp"

#include <pcl/PointIndices.h>

namespace autoware::ground_segmentation
{

void GridGroundFilter::convert()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const size_t in_cloud_data_size = in_cloud_->data.size();
  const size_t in_cloud_point_step = in_cloud_->point_step;

  for (size_t data_index = 0; data_index + in_cloud_point_step <= in_cloud_data_size;
       data_index += in_cloud_point_step) {
    // Get Point
    pcl::PointXYZ input_point;
    data_accessor_.getPoint(in_cloud_, data_index, input_point);
    grid_ptr_->addPoint(input_point.x, input_point.y, data_index);
  }
}

void GridGroundFilter::preprocess()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // eliminate empty cells from connection for efficiency
  grid_ptr_->setGridConnections();
}

bool GridGroundFilter::recursiveSearch(
  const int check_idx, const int search_cnt, std::vector<int> & idx) const
{
  // recursive search
  if (check_idx < 0) {
    return false;
  }
  if (search_cnt == 0) {
    return true;
  }
  const auto & check_cell = grid_ptr_->getCell(check_idx);
  if (check_cell.has_ground_) {
    // the cell has ground, add the index to the list, and search previous cell
    idx.push_back(check_idx);
    return recursiveSearch(check_cell.scan_grid_root_idx_, search_cnt - 1, idx);
  }
  // if the cell does not have ground, search previous cell
  return recursiveSearch(check_cell.scan_grid_root_idx_, search_cnt, idx);
}

void GridGroundFilter::fitLineFromGndGrid(const std::vector<int> & idx, float & a, float & b) const
{
  // if the idx is empty, the line is not defined
  if (idx.empty()) {
    a = 0.0f;
    b = 0.0f;
    return;
  }
  // if the idx is length of 1, the line is zero-crossing line
  if (idx.size() == 1) {
    const auto & cell = grid_ptr_->getCell(idx.front());
    a = cell.avg_height_ / cell.avg_radius_;
    b = 0.0f;
    return;
  }
  // calculate the line by least square method
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xy = 0.0f;
  float sum_x2 = 0.0f;
  for (const auto & i : idx) {
    const auto & cell = grid_ptr_->getCell(i);
    sum_x += cell.avg_radius_;
    sum_y += cell.avg_height_;
    sum_xy += cell.avg_radius_ * cell.avg_height_;
    sum_x2 += cell.avg_radius_ * cell.avg_radius_;
  }
  const float n = static_cast<float>(idx.size());
  a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
  // limit gradient
  a = std::clamp(a, -param_.global_slope_max_ratio, param_.global_slope_max_ratio);
  b = (sum_y - a * sum_x) / n;
}

void GridGroundFilter::initializeGround(pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto grid_size = grid_ptr_->getGridSize();
  // loop over grid cells
  for (size_t idx = 0; idx < grid_size; idx++) {
    auto & cell = grid_ptr_->getCell(idx);
    // if the cell is empty, skip
    if (cell.getPointNum() == 0) {
      continue;
    }
    if (cell.is_ground_initialized_) {
      continue;
    }

    // check scan root grid
    if (cell.scan_grid_root_idx_ >= 0) {
      const Cell & prev_cell = grid_ptr_->getCell(cell.scan_grid_root_idx_);
      if (prev_cell.is_ground_initialized_) {
        cell.is_ground_initialized_ = true;
        continue;
      }
    }

    // initialize ground in this cell
    const auto num_points = static_cast<size_t>(cell.getPointNum());

    bool is_ground_found = false;
    PointsCentroid ground_bin;

    for (size_t j = 0; j < num_points; ++j) {
      const auto & pt_idx = cell.point_indices_[j];
      pcl::PointXYZ point;
      data_accessor_.getPoint(in_cloud_, pt_idx, point);
      const float x_fixed = point.x - param_.virtual_lidar_x;
      const float y_fixed = point.y - param_.virtual_lidar_y;
      const float radius = std::sqrt(x_fixed * x_fixed + y_fixed * y_fixed);

      const float global_slope_ratio = point.z / radius;
      if (
        global_slope_ratio >= param_.global_slope_max_ratio &&
        point.z > param_.non_ground_height_threshold) {
        // this point is obstacle
        out_no_ground_indices.indices.push_back(pt_idx);
      } else if (
        abs(global_slope_ratio) < param_.global_slope_max_ratio &&
        abs(point.z) < param_.non_ground_height_threshold) {
        // this point is ground
        ground_bin.addPoint(radius, point.z, pt_idx);
        is_ground_found = true;
      }
    }
    cell.is_processed_ = true;
    cell.has_ground_ = is_ground_found;
    if (is_ground_found) {
      cell.is_ground_initialized_ = true;
      ground_bin.processAverage();
      cell.avg_height_ = ground_bin.getAverageHeight();
      cell.avg_radius_ = ground_bin.getAverageRadius();
      cell.max_height_ = ground_bin.getMaxHeight();
      cell.min_height_ = ground_bin.getMinHeight();
      cell.gradient_ = cell.avg_height_ / cell.avg_radius_;
      cell.intercept_ = 0.0f;
    } else {
      cell.is_ground_initialized_ = false;
    }
  }
}

void GridGroundFilter::SegmentContinuousCell(
  const Cell & cell, PointsCentroid & ground_bin, pcl::PointIndices & out_no_ground_indices)
{
  const Cell & prev_cell = grid_ptr_->getCell(cell.scan_grid_root_idx_);
  const float gradient =
    std::clamp(cell.gradient_, -param_.global_slope_max_ratio, param_.global_slope_max_ratio);
  const float local_thresh_angle_ratio = std::tan(DEG2RAD(5.0));

  // loop over points in the cell
  const auto num_points = static_cast<size_t>(cell.getPointNum());
  for (size_t j = 0; j < num_points; ++j) {
    const auto & pt_idx = cell.point_indices_[j];
    pcl::PointXYZ point;
    data_accessor_.getPoint(in_cloud_, pt_idx, point);

    // 1. height is out-of-range
    const float delta_z = point.z - prev_cell.avg_height_;
    if (delta_z > param_.detection_range_z_max) {
      // this point is out-of-range
      continue;
    }

    // 2. the angle is exceed the global slope threshold
    const float x_fixed = point.x - param_.virtual_lidar_x;
    const float y_fixed = point.y - param_.virtual_lidar_y;
    const float radius = std::sqrt(x_fixed * x_fixed + y_fixed * y_fixed);
    const float global_slope_ratio = point.z / radius;
    if (global_slope_ratio > param_.global_slope_max_ratio) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }

    // 3. local slope
    const float delta_radius = radius - prev_cell.avg_radius_;
    const float local_slope_ratio = delta_z / delta_radius;
    if (abs(local_slope_ratio) < param_.local_slope_max_ratio) {
      // this point is ground
      ground_bin.addPoint(radius, point.z, pt_idx);
      // go to the next point
      continue;
    }

    // 3. height from the estimated ground
    const float next_gnd_z = gradient * radius + cell.intercept_;
    const float gnd_z_local_thresh = local_thresh_angle_ratio * delta_radius;
    const float delta_gnd_z = point.z - next_gnd_z;
    const float gnd_z_threshold = param_.non_ground_height_threshold + gnd_z_local_thresh;
    if (delta_gnd_z > gnd_z_threshold) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
    if (abs(delta_gnd_z) <= gnd_z_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, point.z, pt_idx);
      // go to the next point
      continue;
    }
  }
}

void GridGroundFilter::SegmentDiscontinuousCell(
  const Cell & cell, PointsCentroid & ground_bin, pcl::PointIndices & out_no_ground_indices)
{
  const Cell & prev_cell = grid_ptr_->getCell(cell.scan_grid_root_idx_);

  // loop over points in the cell
  const auto num_points = static_cast<size_t>(cell.getPointNum());
  for (size_t j = 0; j < num_points; ++j) {
    const auto & pt_idx = cell.point_indices_[j];
    pcl::PointXYZ point;
    data_accessor_.getPoint(in_cloud_, pt_idx, point);

    // 1. height is out-of-range
    const float delta_avg_z = point.z - prev_cell.avg_height_;
    if (delta_avg_z > param_.detection_range_z_max) {
      // this point is out-of-range
      continue;
    }

    // 2. the angle is exceed the global slope threshold
    const float x_fixed = point.x - param_.virtual_lidar_x;
    const float y_fixed = point.y - param_.virtual_lidar_y;
    const float radius = std::sqrt(x_fixed * x_fixed + y_fixed * y_fixed);
    const float global_slope_ratio = point.z / radius;
    if (global_slope_ratio > param_.global_slope_max_ratio) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
    // 3. local slope
    const float delta_radius = radius - prev_cell.avg_radius_;
    const float local_slope_ratio = delta_avg_z / delta_radius;
    if (abs(local_slope_ratio) < param_.local_slope_max_ratio) {
      // this point is ground
      ground_bin.addPoint(radius, point.z, pt_idx);
      // go to the next point
      continue;
    }
    // 4. height from the estimated ground
    if (abs(delta_avg_z) < param_.non_ground_height_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, point.z, pt_idx);
      // go to the next point
      continue;
    }
    const float delta_max_z = point.z - prev_cell.max_height_;
    if (abs(delta_max_z) < param_.non_ground_height_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, point.z, pt_idx);
      // go to the next point
      continue;
    }
    // 5. obstacle from local slope
    if (local_slope_ratio >= param_.local_slope_max_ratio) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
  }
}

void GridGroundFilter::SegmentBreakCell(
  const Cell & cell, PointsCentroid & ground_bin, pcl::PointIndices & out_no_ground_indices)
{
  const Cell & prev_cell = grid_ptr_->getCell(cell.scan_grid_root_idx_);

  // loop over points in the cell
  const auto num_points = static_cast<size_t>(cell.getPointNum());
  for (size_t j = 0; j < num_points; ++j) {
    const auto & pt_idx = cell.point_indices_[j];
    pcl::PointXYZ point;
    data_accessor_.getPoint(in_cloud_, pt_idx, point);

    // 1. height is out-of-range
    const float delta_z = point.z - prev_cell.avg_height_;
    if (delta_z > param_.detection_range_z_max) {
      // this point is out-of-range
      continue;
    }

    // 2. the angle is exceed the global slope threshold
    const float x_fixed = point.x - param_.virtual_lidar_x;
    const float y_fixed = point.y - param_.virtual_lidar_y;
    const float radius = std::sqrt(x_fixed * x_fixed + y_fixed * y_fixed);
    const float global_slope_ratio = point.z / radius;
    if (global_slope_ratio > param_.global_slope_max_ratio) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }

    // 3. the point is over discontinuous grid
    const float delta_radius = radius - prev_cell.avg_radius_;
    const float local_slope_ratio = delta_z / delta_radius;
    if (abs(local_slope_ratio) < param_.global_slope_max_ratio) {
      // this point is ground
      ground_bin.addPoint(radius, point.z, pt_idx);
      // go to the next point
      continue;
    }
    if (local_slope_ratio >= param_.global_slope_max_ratio) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
  }
}

void GridGroundFilter::classify(pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto grid_size = grid_ptr_->getGridSize();
  // loop over grid cells
  for (size_t idx = 0; idx < grid_size; idx++) {
    auto & cell = grid_ptr_->getCell(idx);
    // if the cell is empty, skip
    if (cell.getPointNum() == 0) {
      continue;
    }
    if (cell.is_processed_) {
      continue;
    }

    bool is_previous_initialized = false;
    // set a cell pointer for the previous cell
    const Cell * prev_cell_ptr;
    // check scan root grid
    if (cell.scan_grid_root_idx_ >= 0) {
      prev_cell_ptr = &(grid_ptr_->getCell(cell.scan_grid_root_idx_));
      if (prev_cell_ptr->is_ground_initialized_) {
        is_previous_initialized = true;
      }
    }

    if (!is_previous_initialized) {
      continue;
    }

    // get current cell gradient and intercept
    std::vector<int> grid_idcs;
    {
      const int search_count = param_.gnd_grid_buffer_size;
      const int check_cell_idx = cell.scan_grid_root_idx_;
      recursiveSearch(check_cell_idx, search_count, grid_idcs);
      // calculate the gradient and intercept by least square method
      float a, b;
      fitLineFromGndGrid(grid_idcs, a, b);
      cell.gradient_ = a;
      cell.intercept_ = b;
    }

    // segment the ground and non-ground points
    enum SegmentationMode { NONE, CONTINUOUS, DISCONTINUOUS, BREAK };
    SegmentationMode mode = SegmentationMode::NONE;
    {
      const int front_radial_id =
        grid_ptr_->getCell(grid_idcs.back()).radial_idx_ + grid_idcs.size();
      const float radial_diff_between_cells = cell.center_radius_ - prev_cell_ptr->center_radius_;

      if (radial_diff_between_cells < param_.gnd_grid_continual_thresh * cell.radial_size_) {
        if (cell.radial_idx_ - front_radial_id < param_.gnd_grid_continual_thresh) {
          mode = SegmentationMode::CONTINUOUS;
        } else {
          mode = SegmentationMode::DISCONTINUOUS;
        }
      } else {
        mode = SegmentationMode::BREAK;
      }
    }

    {
      std::unique_ptr<ScopedTimeTrack> sub_st_ptr;
      if (time_keeper_)
        sub_st_ptr = std::make_unique<ScopedTimeTrack>("segmentation", *time_keeper_);

      PointsCentroid ground_bin;
      if (mode == SegmentationMode::CONTINUOUS) {
        SegmentContinuousCell(cell, ground_bin, out_no_ground_indices);
      } else if (mode == SegmentationMode::DISCONTINUOUS) {
        SegmentDiscontinuousCell(cell, ground_bin, out_no_ground_indices);
      } else if (mode == SegmentationMode::BREAK) {
        SegmentBreakCell(cell, ground_bin, out_no_ground_indices);
      }

      // recheck ground bin
      if (
        param_.use_recheck_ground_cluster && cell.avg_radius_ > param_.grid_mode_switch_radius &&
        ground_bin.getGroundPointNum() > 0) {
        // recheck the ground cluster
        float reference_height = 0;
        if (param_.use_lowest_point) {
          reference_height = ground_bin.getMinHeightOnly();
        } else {
          ground_bin.processAverage();
          reference_height = ground_bin.getAverageHeight();
        }
        const float threshold = reference_height + param_.non_ground_height_threshold;
        const std::vector<size_t> & gnd_indices = ground_bin.getIndicesRef();
        const std::vector<float> & height_list = ground_bin.getHeightListRef();
        for (size_t j = 0; j < height_list.size(); ++j) {
          if (height_list.at(j) >= threshold) {
            // fill the non-ground indices
            out_no_ground_indices.indices.push_back(gnd_indices.at(j));
            // mark the point as non-ground
            ground_bin.is_ground_list.at(j) = false;
          }
        }
      }

      // finalize current cell, update the cell ground information
      if (ground_bin.getGroundPointNum() > 0) {
        ground_bin.processAverage();
        cell.avg_height_ = ground_bin.getAverageHeight();
        cell.avg_radius_ = ground_bin.getAverageRadius();
        cell.max_height_ = ground_bin.getMaxHeight();
        cell.min_height_ = ground_bin.getMinHeight();
        cell.has_ground_ = true;
      } else {
        // copy previous cell
        cell.avg_radius_ = prev_cell_ptr->avg_radius_;
        cell.avg_height_ = prev_cell_ptr->avg_height_;
        cell.max_height_ = prev_cell_ptr->max_height_;
        cell.min_height_ = prev_cell_ptr->min_height_;

        cell.has_ground_ = false;
      }

      cell.is_processed_ = true;
    }
  }
}

void GridGroundFilter::process(
  const PointCloud2ConstPtr & in_cloud, pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // set input cloud
  in_cloud_ = in_cloud;

  // clear the output indices
  out_no_ground_indices.indices.clear();

  // reset grid cells
  grid_ptr_->resetCells();

  // 1. assign points to grid cells
  convert();

  // 2. cell preprocess
  preprocess();

  // 3. initialize ground
  initializeGround(out_no_ground_indices);

  // 4. classify point cloud
  classify(out_no_ground_indices);
}

}  // namespace autoware::ground_segmentation
