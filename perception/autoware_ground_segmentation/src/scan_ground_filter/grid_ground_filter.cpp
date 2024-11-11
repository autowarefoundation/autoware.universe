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

namespace autoware::ground_segmentation
{

void GridGroundFilter::convert(const PointCloud2ConstPtr & in_cloud)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const size_t in_cloud_data_size = in_cloud->data.size();
  const size_t in_cloud_point_step = in_cloud->point_step;

  for (size_t data_index = 0; data_index + in_cloud_point_step <= in_cloud_data_size;
       data_index += in_cloud_point_step) {
    // Get Point
    pcl::PointXYZ input_point;
    data_accessor_.getPoint(in_cloud, data_index, input_point);
    grid_ptr_->addPoint(input_point.x, input_point.y, data_index);
  }
}

void GridGroundFilter::preprocess()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  grid_ptr_->setGridConnections();
  // grid_ptr_->setGridStatistics();
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
  if (!check_cell.has_ground_) {
    // if the cell does not have ground, search previous cell
    return recursiveSearch(check_cell.prev_grid_idx_, search_cnt, idx);
  }
  // the cell has ground, add the index to the list, and search previous cell
  idx.push_back(check_idx);
  return recursiveSearch(check_cell.prev_grid_idx_, search_cnt - 1, idx);
}

void GridGroundFilter::fitLineFromGndGrid(const std::vector<int> & idx, float & a, float & b) const
{
  // if the idx is empty, the line is not defined
  if (idx.empty()) {
    a = 0.0f;
    b = 0.0f;
    return;
  }
  // if the idx is length of 1, the line is horizontal
  if (idx.size() == 1) {
    a = 0.0f;
    b = grid_ptr_->getCell(idx.front()).avg_height_;
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
  b = (sum_y - a * sum_x) / n;
}

void GridGroundFilter::classify(
  const PointCloud2ConstPtr & in_cloud, pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // [new grid] run ground segmentation
  out_no_ground_indices.indices.clear();
  const auto grid_size = grid_ptr_->getGridSize();
  // loop over grid cells
  for (size_t idx = 0; idx < grid_size; idx++) {
    auto & cell = grid_ptr_->getCell(idx);
    if (cell.is_processed_) {
      continue;
    }
    // if the cell is empty, skip
    if (cell.getPointNum() == 0) {
      continue;
    }

    // iterate over points in the grid cell
    const auto num_points = static_cast<size_t>(cell.getPointNum());

    bool is_previous_initialized = false;
    // set a cell pointer for the previous cell
    const Cell * prev_cell_ptr;
    // check prev grid only exist
    if (cell.prev_grid_idx_ >= 0) {
      prev_cell_ptr = &(grid_ptr_->getCell(cell.prev_grid_idx_));
      if (prev_cell_ptr->is_ground_initialized_) {
        is_previous_initialized = true;
      }
    }

    if (!is_previous_initialized) {
      // if the previous cell is not processed or do not have ground,
      // try initialize ground in this cell
      bool is_ground_found = false;
      PointsCentroid ground_bin;

      for (size_t j = 0; j < num_points; ++j) {
        const auto & pt_idx = cell.point_indices_[j];
        pcl::PointXYZ point;
        data_accessor_.getPoint(in_cloud, pt_idx, point);
        const float radius = std::hypot(point.x, point.y);
        const float global_slope_ratio = point.z / radius;
        if (
          global_slope_ratio >= param_.global_slope_max_ratio &&
          point.z < param_.non_ground_height_threshold) {
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
      }

      // go to the next cell
      continue;
    } else {
      // inherit the previous cell information
      cell.is_ground_initialized_ = true;
      // continue to the ground segmentation
    }

    // get current cell gradient and intercept
    std::vector<int> grid_idcs;
    {
      const int search_count = param_.gnd_grid_buffer_size;
      int check_cell_idx = cell.prev_grid_idx_;
      recursiveSearch(check_cell_idx, search_count, grid_idcs);
      if (grid_idcs.size() > 0) {
        // calculate the gradient and intercept by least square method
        float a, b;
        fitLineFromGndGrid(grid_idcs, a, b);
        cell.gradient_ = a;
        cell.intercept_ = b;
      } else {
        // initialization failed. which should not happen. print error message
        std::cerr << "Failed to initialize ground grid at cell index: " << cell.grid_idx_
                  << std::endl;
      }
    }

    // segment the ground and non-ground points
    bool is_continuous = true;
    bool is_discontinuous = false;
    bool is_break = false;
    {
      const int front_radial_id =
        grid_ptr_->getCell(grid_idcs.front()).radial_idx_ + grid_idcs.size();
      const float radial_diff_between_cells = cell.center_radius_ - prev_cell_ptr->center_radius_;

      if (radial_diff_between_cells < param_.gnd_grid_continual_thresh * cell.radial_size_) {
        if (cell.grid_idx_ - front_radial_id < param_.gnd_grid_continual_thresh) {
          is_continuous = true;
          is_discontinuous = false;
          is_break = false;
        } else {
          is_continuous = false;
          is_discontinuous = true;
          is_break = false;
        }
      } else {
        is_continuous = false;
        is_discontinuous = false;
        is_break = true;
      }
    }

    {
      PointsCentroid ground_bin;
      for (size_t j = 0; j < num_points; ++j) {
        const auto & pt_idx = cell.point_indices_[j];
        pcl::PointXYZ point;
        data_accessor_.getPoint(in_cloud, pt_idx, point);

        // 1. height is out-of-range
        if (point.z - prev_cell_ptr->avg_height_ > param_.detection_range_z_max) {
          // this point is out-of-range
          continue;
        }

        // 2. the angle is exceed the global slope threshold
        const float radius = std::hypot(point.x, point.y);
        const float global_slope_ratio = point.z / radius;
        if (global_slope_ratio > param_.global_slope_max_ratio) {
          // this point is obstacle
          out_no_ground_indices.indices.push_back(pt_idx);
          // go to the next point
          continue;
        }

        // 3. the point is continuous with the previous grid
        if (is_continuous) {
          // 3-a. local slope
          const float delta_z = point.z - prev_cell_ptr->avg_height_;
          const float delta_radius = radius - prev_cell_ptr->avg_radius_;
          const float local_slope_ratio = delta_z / delta_radius;
          if (abs(local_slope_ratio) < param_.local_slope_max_ratio) {
            // this point is ground
            ground_bin.addPoint(radius, point.z, pt_idx);
            // go to the next point
            continue;
          }

          // 3-b. mean of grid buffer(filtering)
          const float gradient = std::clamp(
            cell.gradient_, -param_.global_slope_max_ratio, param_.global_slope_max_ratio);
          const float next_gnd_z = gradient * radius + cell.intercept_;
          const float gnd_z_local_thresh =
            std::tan(DEG2RAD(5.0)) * (radius - prev_cell_ptr->avg_radius_);
          if (
            abs(point.z - next_gnd_z) <= param_.non_ground_height_threshold + gnd_z_local_thresh) {
            // this point is ground
            ground_bin.addPoint(radius, point.z, pt_idx);
            // go to the next point
            continue;
          }

          // 3-c. the point is non-ground
          if (point.z - next_gnd_z >= param_.non_ground_height_threshold + gnd_z_local_thresh) {
            // this point is obstacle
            out_no_ground_indices.indices.push_back(pt_idx);
            // go to the next point
            continue;
          }
        }

        // 4. the point is discontinuous with the previous grid
        if (is_discontinuous) {
          const float delta_avg_z = point.z - prev_cell_ptr->avg_height_;
          if (abs(delta_avg_z) < param_.non_ground_height_threshold) {
            // this point is ground
            ground_bin.addPoint(radius, point.z, pt_idx);
            // go to the next point
            continue;
          }
          const float delta_max_z = point.z - prev_cell_ptr->max_height_;
          if (abs(delta_max_z) < param_.non_ground_height_threshold) {
            // this point is ground
            ground_bin.addPoint(radius, point.z, pt_idx);
            // go to the next point
            continue;
          }
          const float delta_radius = radius - prev_cell_ptr->avg_radius_;
          const float local_slope_ratio = delta_avg_z / delta_radius;
          if (abs(local_slope_ratio) < param_.local_slope_max_ratio) {
            // this point is ground
            ground_bin.addPoint(radius, point.z, pt_idx);
            // go to the next point
            continue;
          }
          if (local_slope_ratio >= param_.local_slope_max_ratio) {
            // this point is obstacle
            out_no_ground_indices.indices.push_back(pt_idx);
            // go to the next point
            continue;
          }
        }

        // 5. the point is break the previous grid
        if (is_break) {
          const float delta_avg_z = point.z - prev_cell_ptr->avg_height_;
          const float delta_radius = radius - prev_cell_ptr->avg_radius_;
          const float local_slope_ratio = delta_avg_z / delta_radius;
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

      // recheck ground bin
      if (ground_bin.getIndicesRef().size() > 0 && param_.use_recheck_ground_cluster) {
        ground_bin.processAverage();
        // recheck the ground cluster
        const float reference_height =
          param_.use_lowest_point ? ground_bin.getMinHeight() : ground_bin.getAverageHeight();
        const std::vector<size_t> & gnd_indices = ground_bin.getIndicesRef();
        const std::vector<float> & height_list = ground_bin.getHeightListRef();
        for (size_t j = 0; j < height_list.size(); ++j) {
          if (height_list.at(j) >= reference_height + param_.non_ground_height_threshold) {
            // fill the non-ground indices
            out_no_ground_indices.indices.push_back(gnd_indices.at(j));
            // remove the point from the ground bin
            // ground_bin.removePoint(j);
          }
        }
      }

      // finalize current cell, update the cell ground information
      if (ground_bin.getIndicesRef().size() > 0) {
        ground_bin.processAverage();
        cell.avg_height_ = ground_bin.getAverageHeight();
        cell.avg_radius_ = ground_bin.getAverageRadius();
        cell.max_height_ = ground_bin.getMaxHeight();
        cell.min_height_ = ground_bin.getMinHeight();
        cell.has_ground_ = true;
      }
      cell.is_processed_ = true;
    }

    // // debug: cell info for all non-empty cells
    // std::cout << "cell index: " << cell.grid_idx_
    //            <<  " number of points: " << cell.getPointNum()
    //             << " has ground: " << cell.has_ground_
    //             << " avg height: " << cell.avg_height_ << " avg radius: " << cell.avg_radius_
    //             << " gradient: " << cell.gradient_ << " intercept: " << cell.intercept_
    //             << std::endl;
  }
}

void GridGroundFilter::process(
  const PointCloud2ConstPtr & in_cloud, pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // reset grid cells
  grid_ptr_->resetCells();

  // 1. convert
  convert(in_cloud);

  // 2. preprocess
  preprocess();

  // 3. classify point cloud
  classify(in_cloud, out_no_ground_indices);
}

}  // namespace autoware::ground_segmentation
