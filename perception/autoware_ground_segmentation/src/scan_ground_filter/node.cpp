// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "node.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::ground_segmentation
{
using autoware::pointcloud_preprocessor::get_param;
using autoware::universe_utils::calcDistance3d;
using autoware::universe_utils::deg2rad;
using autoware::universe_utils::normalizeDegree;
using autoware::universe_utils::normalizeRadian;
using autoware::universe_utils::ScopedTimeTrack;
using autoware::vehicle_info_utils::VehicleInfoUtils;

ScanGroundFilterComponent::ScanGroundFilterComponent(const rclcpp::NodeOptions & options)
: autoware::pointcloud_preprocessor::Filter("ScanGroundFilter", options)
{
  // set initial parameters
  {
    low_priority_region_x_ = declare_parameter<float>("low_priority_region_x");
    detection_range_z_max_ = declare_parameter<float>("detection_range_z_max");
    center_pcl_shift_ = declare_parameter<float>("center_pcl_shift");
    non_ground_height_threshold_ = declare_parameter<float>("non_ground_height_threshold");
    grid_mode_switch_radius_ = declare_parameter<float>("grid_mode_switch_radius");

    grid_size_m_ = declare_parameter<float>("grid_size_m");
    gnd_grid_buffer_size_ = declare_parameter<int>("gnd_grid_buffer_size");
    elevation_grid_mode_ = declare_parameter<bool>("elevation_grid_mode");
    global_slope_max_angle_rad_ = deg2rad(declare_parameter<float>("global_slope_max_angle_deg"));
    local_slope_max_angle_rad_ = deg2rad(declare_parameter<float>("local_slope_max_angle_deg"));
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    radial_divider_angle_rad_ = deg2rad(declare_parameter<float>("radial_divider_angle_deg"));
    split_points_distance_tolerance_ = declare_parameter<float>("split_points_distance_tolerance");
    split_points_distance_tolerance_square_ =
      split_points_distance_tolerance_ * split_points_distance_tolerance_;
    split_height_distance_ = declare_parameter<float>("split_height_distance");
    use_virtual_ground_point_ = declare_parameter<bool>("use_virtual_ground_point");
    use_recheck_ground_cluster_ = declare_parameter<bool>("use_recheck_ground_cluster");
    use_lowest_point_ = declare_parameter<bool>("use_lowest_point");
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    vehicle_info_ = VehicleInfoUtils(*this).getVehicleInfo();

    grid_mode_switch_grid_id_ =
      grid_mode_switch_radius_ / grid_size_m_;  // changing the mode of grid division
    virtual_lidar_z_ = vehicle_info_.vehicle_height_m;
    grid_mode_switch_angle_rad_ = std::atan2(grid_mode_switch_radius_, virtual_lidar_z_);

    grid_size_rad_ =
      normalizeRadian(std::atan2(grid_mode_switch_radius_ + grid_size_m_, virtual_lidar_z_)) -
      normalizeRadian(std::atan2(grid_mode_switch_radius_, virtual_lidar_z_));
    tan_grid_size_rad_ = std::tan(grid_size_rad_);
    data_offset_initialized_ = false;
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ScanGroundFilterComponent::onParameter, this, _1));

  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, "scan_ground_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");

    bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
    if (use_time_keeper) {
      detailed_processing_time_publisher_ =
        this->create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
          "~/debug/processing_time_detail_ms", 1);
      auto time_keeper = autoware::universe_utils::TimeKeeper(detailed_processing_time_publisher_);
      time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(time_keeper);
    }
  }
}

inline void ScanGroundFilterComponent::set_field_index_offsets(const PointCloud2ConstPtr & input)
{
  data_offset_x_ = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  data_offset_y_ = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  data_offset_z_ = input->fields[pcl::getFieldIndex(*input, "z")].offset;
  int intensity_index = pcl::getFieldIndex(*input, "intensity");
  if (intensity_index != -1) {
    data_offset_intensity_ = input->fields[intensity_index].offset;
    intensity_type_ = input->fields[intensity_index].datatype;
  } else {
    data_offset_intensity_ = -1;
  }
  data_offset_initialized_ = true;
}

inline void ScanGroundFilterComponent::get_point_from_data_index(
  const PointCloud2ConstPtr & input, const size_t data_index, pcl::PointXYZ & point)
{
  point.x = *reinterpret_cast<const float *>(&input->data[data_index + data_offset_x_]);
  point.y = *reinterpret_cast<const float *>(&input->data[data_index + data_offset_y_]);
  point.z = *reinterpret_cast<const float *>(&input->data[data_index + data_offset_z_]);
}

void ScanGroundFilterComponent::convertPointcloudGridScan(
  const PointCloud2ConstPtr & in_cloud, std::vector<PointCloudVector> & out_radial_ordered_points)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  out_radial_ordered_points.resize(radial_dividers_num_);
  PointData current_point;

  const auto inv_radial_divider_angle_rad = 1.0f / radial_divider_angle_rad_;
  const auto inv_grid_size_rad = 1.0f / grid_size_rad_;
  const auto inv_grid_size_m = 1.0f / grid_size_m_;

  const auto grid_id_offset =
    grid_mode_switch_grid_id_ - grid_mode_switch_angle_rad_ * inv_grid_size_rad;
  const auto x_shift = vehicle_info_.wheel_base_m / 2.0f + center_pcl_shift_;

  const size_t in_cloud_data_size = in_cloud->data.size();
  const size_t in_cloud_point_step = in_cloud->point_step;

  {  // grouping pointcloud by its azimuth angle
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("azimuth_angle_grouping", *time_keeper_);

    size_t point_index = 0;
    pcl::PointXYZ input_point;
    for (size_t data_index = 0; data_index + in_cloud_point_step <= in_cloud_data_size;
         data_index += in_cloud_point_step) {
      get_point_from_data_index(in_cloud, data_index, input_point);

      auto x{input_point.x - x_shift};  // base on front wheel center
      auto radius{static_cast<float>(std::hypot(x, input_point.y))};
      auto theta{normalizeRadian(std::atan2(x, input_point.y), 0.0)};

      // divide by azimuth angle
      auto radial_div{static_cast<size_t>(std::floor(theta * inv_radial_divider_angle_rad))};
      uint16_t grid_id = 0;
      // TODO(technolojin): radius to grid_id conversion can be a function for better readability
      // and cached for performance
      if (radius <= grid_mode_switch_radius_) {
        grid_id = static_cast<uint16_t>(radius * inv_grid_size_m);
      } else {
        auto gamma{normalizeRadian(std::atan2(radius, virtual_lidar_z_), 0.0f)};
        grid_id = grid_id_offset + gamma * inv_grid_size_rad;
      }
      current_point.grid_id = grid_id;
      current_point.radius = radius;
      current_point.point_state = PointLabel::INIT;
      current_point.orig_index = point_index;

      // radial divisions
      out_radial_ordered_points[radial_div].emplace_back(current_point);
      ++point_index;
    }
  }

  {  // sorting pointcloud by distance, on each azimuth angle group
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_) inner_st_ptr = std::make_unique<ScopedTimeTrack>("sort", *time_keeper_);

    for (size_t i = 0; i < radial_dividers_num_; ++i) {
      std::sort(
        out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
        [](const PointData & a, const PointData & b) { return a.radius < b.radius; });
    }
  }
}

void ScanGroundFilterComponent::convertPointcloud(
  const PointCloud2ConstPtr & in_cloud, std::vector<PointCloudVector> & out_radial_ordered_points)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  out_radial_ordered_points.resize(radial_dividers_num_);
  PointData current_point;

  const auto inv_radial_divider_angle_rad = 1.0f / radial_divider_angle_rad_;

  const size_t in_cloud_data_size = in_cloud->data.size();
  const size_t in_cloud_point_step = in_cloud->point_step;

  {  // grouping pointcloud by its azimuth angle
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("azimuth_angle_grouping", *time_keeper_);

    size_t point_index = 0;
    pcl::PointXYZ input_point;
    for (size_t data_index = 0; data_index + in_cloud_point_step <= in_cloud_data_size;
         data_index += in_cloud_point_step) {
      // Point
      get_point_from_data_index(in_cloud, data_index, input_point);

      auto radius{static_cast<float>(std::hypot(input_point.x, input_point.y))};
      auto theta{normalizeRadian(std::atan2(input_point.x, input_point.y), 0.0)};
      auto radial_div{static_cast<size_t>(std::floor(theta * inv_radial_divider_angle_rad))};

      current_point.radius = radius;
      current_point.point_state = PointLabel::INIT;
      current_point.orig_index = point_index;

      // radial divisions
      out_radial_ordered_points[radial_div].emplace_back(current_point);
      ++point_index;
    }
  }

  {  // sorting pointcloud by distance, on each azimuth angle group
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_) inner_st_ptr = std::make_unique<ScopedTimeTrack>("sort", *time_keeper_);

    for (size_t i = 0; i < radial_dividers_num_; ++i) {
      std::sort(
        out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
        [](const PointData & a, const PointData & b) { return a.radius < b.radius; });
    }
  }
}

void ScanGroundFilterComponent::calcVirtualGroundOrigin(pcl::PointXYZ & point)
{
  point.x = vehicle_info_.wheel_base_m;
  point.y = 0;
  point.z = 0;
}

inline float ScanGroundFilterComponent::calcGridSize(const PointData & pd)
{
  float grid_size = grid_size_m_;
  uint16_t back_steps_num = 1;

  if (
    pd.radius > grid_mode_switch_radius_ &&
    pd.grid_id > grid_mode_switch_grid_id_ + back_steps_num) {
    // equivalent to grid_size = (std::tan(gamma) - std::tan(gamma - grid_size_rad_)) *
    // virtual_lidar_z_ when gamma = normalizeRadian(std::atan2(radius, virtual_lidar_z_), 0.0f)
    grid_size = pd.radius - (pd.radius - tan_grid_size_rad_ * virtual_lidar_z_) /
                              (1 + pd.radius * tan_grid_size_rad_ / virtual_lidar_z_);
  }
  return grid_size;
}

void ScanGroundFilterComponent::initializeFirstGndGrids(
  const float h, const float r, const uint16_t id, std::vector<GridCenter> & gnd_grids)
{
  GridCenter curr_gnd_grid;
  for (int ind_grid = id - 1 - gnd_grid_buffer_size_; ind_grid < id - 1; ++ind_grid) {
    float ind_gnd_z = ind_grid - id + 1 + gnd_grid_buffer_size_;
    ind_gnd_z *= h / static_cast<float>(gnd_grid_buffer_size_);

    float ind_gnd_radius = ind_grid - id + 1 + gnd_grid_buffer_size_;
    ind_gnd_radius *= r / static_cast<float>(gnd_grid_buffer_size_);

    curr_gnd_grid.radius = ind_gnd_radius;
    curr_gnd_grid.avg_height = ind_gnd_z;
    curr_gnd_grid.max_height = ind_gnd_z;
    curr_gnd_grid.grid_id = ind_grid;
    gnd_grids.push_back(curr_gnd_grid);
  }
}

void ScanGroundFilterComponent::checkContinuousGndGrid(
  PointData & pd, const pcl::PointXYZ & point_curr, const std::vector<GridCenter> & gnd_grids_list)
{
  float next_gnd_z = 0.0f;
  float curr_gnd_slope_ratio = 0.0f;
  float gnd_buff_z_mean = 0.0f;
  float gnd_buff_radius = 0.0f;

  for (auto it = gnd_grids_list.end() - gnd_grid_buffer_size_ - 1; it < gnd_grids_list.end() - 1;
       ++it) {
    gnd_buff_radius += it->radius;
    gnd_buff_z_mean += it->avg_height;
  }

  gnd_buff_radius /= static_cast<float>(gnd_grid_buffer_size_ - 1);
  gnd_buff_z_mean /= static_cast<float>(gnd_grid_buffer_size_ - 1);

  float tmp_delta_mean_z = gnd_grids_list.back().avg_height - gnd_buff_z_mean;
  float tmp_delta_radius = gnd_grids_list.back().radius - gnd_buff_radius;

  curr_gnd_slope_ratio = tmp_delta_mean_z / tmp_delta_radius;
  curr_gnd_slope_ratio = curr_gnd_slope_ratio < -global_slope_max_ratio_ ? -global_slope_max_ratio_
                                                                         : curr_gnd_slope_ratio;
  curr_gnd_slope_ratio =
    curr_gnd_slope_ratio > global_slope_max_ratio_ ? global_slope_max_ratio_ : curr_gnd_slope_ratio;

  next_gnd_z = curr_gnd_slope_ratio * (pd.radius - gnd_buff_radius) + gnd_buff_z_mean;

  float gnd_z_local_thresh = std::tan(DEG2RAD(5.0)) * (pd.radius - gnd_grids_list.back().radius);

  tmp_delta_mean_z = point_curr.z - (gnd_grids_list.end() - 2)->avg_height;
  tmp_delta_radius = pd.radius - (gnd_grids_list.end() - 2)->radius;
  float local_slope_ratio = tmp_delta_mean_z / tmp_delta_radius;
  if (
    abs(point_curr.z - next_gnd_z) <= non_ground_height_threshold_ + gnd_z_local_thresh ||
    abs(local_slope_ratio) <= local_slope_max_ratio_) {
    pd.point_state = PointLabel::GROUND;
  } else if (point_curr.z - next_gnd_z > non_ground_height_threshold_ + gnd_z_local_thresh) {
    pd.point_state = PointLabel::NON_GROUND;
  }
}

void ScanGroundFilterComponent::checkDiscontinuousGndGrid(
  PointData & pd, const pcl::PointXYZ & point_curr, const std::vector<GridCenter> & gnd_grids_list)
{
  float tmp_delta_max_z = point_curr.z - gnd_grids_list.back().max_height;
  float tmp_delta_avg_z = point_curr.z - gnd_grids_list.back().avg_height;
  float tmp_delta_radius = pd.radius - gnd_grids_list.back().radius;
  float local_slope_ratio = tmp_delta_avg_z / tmp_delta_radius;

  if (
    abs(local_slope_ratio) < local_slope_max_ratio_ ||
    abs(tmp_delta_avg_z) < non_ground_height_threshold_ ||
    abs(tmp_delta_max_z) < non_ground_height_threshold_) {
    pd.point_state = PointLabel::GROUND;
  } else if (local_slope_ratio > global_slope_max_ratio_) {
    pd.point_state = PointLabel::NON_GROUND;
  }
}

void ScanGroundFilterComponent::checkBreakGndGrid(
  PointData & pd, const pcl::PointXYZ & point_curr, const std::vector<GridCenter> & gnd_grids_list)
{
  float tmp_delta_avg_z = point_curr.z - gnd_grids_list.back().avg_height;
  float tmp_delta_radius = pd.radius - gnd_grids_list.back().radius;
  float local_slope_ratio = tmp_delta_avg_z / tmp_delta_radius;
  if (abs(local_slope_ratio) < global_slope_max_ratio_) {
    pd.point_state = PointLabel::GROUND;
  } else if (local_slope_ratio > global_slope_max_ratio_) {
    pd.point_state = PointLabel::NON_GROUND;
  }
}

void ScanGroundFilterComponent::recheckGroundCluster(
  const PointsCentroid & gnd_cluster, const float non_ground_threshold, const bool use_lowest_point,
  pcl::PointIndices & non_ground_indices)
{
  float reference_height =
    use_lowest_point ? gnd_cluster.getMinHeight() : gnd_cluster.getAverageHeight();
  const pcl::PointIndices & gnd_indices = gnd_cluster.getIndicesRef();
  const std::vector<float> & height_list = gnd_cluster.getHeightListRef();
  for (size_t i = 0; i < height_list.size(); ++i) {
    if (height_list.at(i) >= reference_height + non_ground_threshold) {
      non_ground_indices.indices.push_back(gnd_indices.indices.at(i));
    }
  }
}

void ScanGroundFilterComponent::classifyPointCloudGridScan(
  const PointCloud2ConstPtr & in_cloud,
  const std::vector<PointCloudVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  out_no_ground_indices.indices.clear();

  // run the classification algorithm for each ray (azimuth division)
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); ++i) {
    PointsCentroid centroid_bin;
    centroid_bin.initialize();
    std::vector<GridCenter> gnd_grids;

    // check empty ray
    if (in_radial_ordered_clouds[i].size() == 0) {
      continue;
    }

    bool initialized_first_gnd_grid = false;
    bool prev_list_init = false;

    PointData pd_curr, pd_prev;
    pcl::PointXYZ point_curr, point_prev;

    pd_curr = in_radial_ordered_clouds[i][0];

    // iterate over the points in the ray
    for (const auto & point : in_radial_ordered_clouds[i]) {
      // set the previous point
      point_prev = point_curr;
      pd_prev = pd_curr;

      // set the current point
      pd_curr = point;
      const size_t data_index = in_cloud->point_step * pd_curr.orig_index;
      get_point_from_data_index(in_cloud, data_index, point_curr);

      // set the thresholds
      const float global_slope_ratio_p = point_curr.z / pd_curr.radius;
      float non_ground_height_threshold_local = non_ground_height_threshold_;
      if (point_curr.x < low_priority_region_x_) {
        non_ground_height_threshold_local =
          non_ground_height_threshold_ * abs(point_curr.x / low_priority_region_x_);
      }

      if (!initialized_first_gnd_grid) {
        // classify first grid's point cloud
        if (
          global_slope_ratio_p >= global_slope_max_ratio_ &&
          point_curr.z > non_ground_height_threshold_local) {
          out_no_ground_indices.indices.push_back(pd_curr.orig_index);
          pd_curr.point_state = PointLabel::NON_GROUND;
        } else if (
          abs(global_slope_ratio_p) < global_slope_max_ratio_ &&
          abs(point_curr.z) < non_ground_height_threshold_local) {
          centroid_bin.addPoint(pd_curr.radius, point_curr.z, pd_curr.orig_index);
          pd_curr.point_state = PointLabel::GROUND;
          // if the gird id is not the initial grid_id, then the first gnd grid is initialized
          initialized_first_gnd_grid = static_cast<bool>(pd_curr.grid_id - pd_prev.grid_id);
        }
        // else, the point is not classified
        continue;
      }

      // initialize gnd_grids based on the initial centroid_bin
      if (!prev_list_init) {
        float h = centroid_bin.getAverageHeight();
        float r = centroid_bin.getAverageRadius();
        initializeFirstGndGrids(h, r, pd_curr.grid_id, gnd_grids);
        prev_list_init = true;
      }

      // finalize the current centroid_bin
      if (pd_curr.grid_id > pd_prev.grid_id && centroid_bin.getAverageRadius() > 0.0) {
        // check if the prev grid have ground point cloud
        if (use_recheck_ground_cluster_) {
          recheckGroundCluster(
            centroid_bin, non_ground_height_threshold_, use_lowest_point_, out_no_ground_indices);
          // centroid_bin is not modified. should be rechecked by out_no_ground_indices?
        }
        // convert the centroid_bin to gnd grid and add it to the gnd_grids
        GridCenter curr_gnd_grid;
        curr_gnd_grid.radius = centroid_bin.getAverageRadius();
        curr_gnd_grid.avg_height = centroid_bin.getAverageHeight();
        curr_gnd_grid.max_height = centroid_bin.getMaxHeight();
        curr_gnd_grid.grid_id = pd_prev.grid_id;
        gnd_grids.push_back(curr_gnd_grid);
        // clear the centroid_bin
        centroid_bin.initialize();
      }

      // 1: height is out-of-range
      if (point_curr.z - gnd_grids.back().avg_height > detection_range_z_max_) {
        pd_curr.point_state = PointLabel::OUT_OF_RANGE;
        continue;
      }

      // 2: continuously non-ground
      float points_xy_distance_square =
        (point_curr.x - point_prev.x) * (point_curr.x - point_prev.x) +
        (point_curr.y - point_prev.y) * (point_curr.y - point_prev.y);
      if (
        pd_prev.point_state == PointLabel::NON_GROUND &&
        points_xy_distance_square < split_points_distance_tolerance_square_ &&
        point_curr.z > point_prev.z) {
        pd_curr.point_state = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(pd_curr.orig_index);
        continue;
      }

      // 3: the angle is exceed the global slope threshold
      if (global_slope_ratio_p > global_slope_max_ratio_) {
        out_no_ground_indices.indices.push_back(pd_curr.orig_index);
        continue;
      }

      uint16_t next_gnd_grid_id_thresh = (gnd_grids.end() - gnd_grid_buffer_size_)->grid_id +
                                         gnd_grid_buffer_size_ + gnd_grid_continual_thresh_;
      float curr_grid_size = calcGridSize(pd_curr);
      if (
        // 4: the point is in the same grid
        pd_curr.grid_id < next_gnd_grid_id_thresh &&
        pd_curr.radius - gnd_grids.back().radius < gnd_grid_continual_thresh_ * curr_grid_size) {
        checkContinuousGndGrid(pd_curr, point_curr, gnd_grids);
      } else if (
        // 5: the point is in the next grid
        pd_curr.radius - gnd_grids.back().radius < gnd_grid_continual_thresh_ * curr_grid_size) {
        checkDiscontinuousGndGrid(pd_curr, point_curr, gnd_grids);
      } else {
        // 6: the point is in the break grid
        checkBreakGndGrid(pd_curr, point_curr, gnd_grids);
      }

      // update the point label and update the ground cluster
      if (pd_curr.point_state == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(pd_curr.orig_index);
      } else if (pd_curr.point_state == PointLabel::GROUND) {
        centroid_bin.addPoint(pd_curr.radius, point_curr.z, pd_curr.orig_index);
      }
      // else, the point is not classified
    }
  }
}

void ScanGroundFilterComponent::classifyPointCloud(
  const PointCloud2ConstPtr & in_cloud, std::vector<PointCloudVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  out_no_ground_indices.indices.clear();

  const pcl::PointXYZ init_ground_point(0, 0, 0);
  pcl::PointXYZ virtual_ground_point(0, 0, 0);
  calcVirtualGroundOrigin(virtual_ground_point);

  // run the classification algorithm for each ray (azimuth division)
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); ++i) {
    float prev_gnd_radius = 0.0f;
    float prev_gnd_slope = 0.0f;
    PointsCentroid ground_cluster, non_ground_cluster;
    PointLabel prev_point_label = PointLabel::INIT;
    pcl::PointXYZ prev_gnd_point(0, 0, 0), point_curr, point_prev;

    // iterate over the points in the ray
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); ++j) {
      float points_distance = 0.0f;
      const float local_slope_max_angle = local_slope_max_angle_rad_;
      point_prev = point_curr;
      auto * p = &in_radial_ordered_clouds[i][j];
      const size_t data_index = in_cloud->point_step * p->orig_index;
      get_point_from_data_index(in_cloud, data_index, point_curr);
      if (j == 0) {
        bool is_front_side = (point_curr.x > virtual_ground_point.x);
        if (use_virtual_ground_point_ && is_front_side) {
          prev_gnd_point = virtual_ground_point;
        } else {
          prev_gnd_point = init_ground_point;
        }
        prev_gnd_radius = std::hypot(prev_gnd_point.x, prev_gnd_point.y);
        prev_gnd_slope = 0.0f;
        ground_cluster.initialize();
        non_ground_cluster.initialize();
        points_distance = calcDistance3d(point_curr, prev_gnd_point);
      } else {
        points_distance = calcDistance3d(point_curr, point_prev);
      }

      float radius_distance_from_gnd = p->radius - prev_gnd_radius;
      float height_from_gnd = point_curr.z - prev_gnd_point.z;
      float height_from_obj = point_curr.z - non_ground_cluster.getAverageHeight();
      bool calculate_slope = false;
      bool is_point_close_to_prev =
        (points_distance <
         (p->radius * radial_divider_angle_rad_ + split_points_distance_tolerance_));

      float global_slope_ratio = point_curr.z / p->radius;
      // check points which is far enough from previous point
      if (global_slope_ratio > global_slope_max_ratio_) {
        p->point_state = PointLabel::NON_GROUND;
        calculate_slope = false;
      } else if (
        (prev_point_label == PointLabel::NON_GROUND) &&
        (std::abs(height_from_obj) >= split_height_distance_)) {
        calculate_slope = true;
      } else if (is_point_close_to_prev && std::abs(height_from_gnd) < split_height_distance_) {
        // close to the previous point, set point follow label
        p->point_state = PointLabel::POINT_FOLLOW;
        calculate_slope = false;
      } else {
        calculate_slope = true;
      }
      if (is_point_close_to_prev) {
        height_from_gnd = point_curr.z - ground_cluster.getAverageHeight();
        radius_distance_from_gnd = p->radius - ground_cluster.getAverageRadius();
      }
      if (calculate_slope) {
        // far from the previous point
        auto local_slope = std::atan2(height_from_gnd, radius_distance_from_gnd);
        if (local_slope - prev_gnd_slope > local_slope_max_angle) {
          // the point is outside of the local slope threshold
          p->point_state = PointLabel::NON_GROUND;
        } else {
          p->point_state = PointLabel::GROUND;
        }
      }

      if (p->point_state == PointLabel::GROUND) {
        ground_cluster.initialize();
        non_ground_cluster.initialize();
      }
      if (p->point_state == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(p->orig_index);
      } else if (  // NOLINT
        (prev_point_label == PointLabel::NON_GROUND) &&
        (p->point_state == PointLabel::POINT_FOLLOW)) {
        p->point_state = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(p->orig_index);
      } else if (  // NOLINT
        (prev_point_label == PointLabel::GROUND) && (p->point_state == PointLabel::POINT_FOLLOW)) {
        p->point_state = PointLabel::GROUND;
      } else {
      }

      // update the ground state
      prev_point_label = p->point_state;
      if (p->point_state == PointLabel::GROUND) {
        prev_gnd_radius = p->radius;
        prev_gnd_point = pcl::PointXYZ(point_curr.x, point_curr.y, point_curr.z);
        ground_cluster.addPoint(p->radius, point_curr.z);
        prev_gnd_slope = ground_cluster.getAverageSlope();
      }
      // update the non ground state
      if (p->point_state == PointLabel::NON_GROUND) {
        non_ground_cluster.addPoint(p->radius, point_curr.z);
      }
    }
  }
}

void ScanGroundFilterComponent::extractObjectPoints(
  const PointCloud2ConstPtr & in_cloud_ptr, const pcl::PointIndices & in_indices,
  PointCloud2 & out_object_cloud)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  size_t output_data_size = 0;

  for (const auto & i : in_indices.indices) {
    std::memcpy(
      &out_object_cloud.data[output_data_size], &in_cloud_ptr->data[i * in_cloud_ptr->point_step],
      in_cloud_ptr->point_step * sizeof(uint8_t));
    output_data_size += in_cloud_ptr->point_step;
  }
}

void ScanGroundFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output,
  [[maybe_unused]] const autoware::pointcloud_preprocessor::TransformInfo & transform_info)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  if (!data_offset_initialized_) {
    set_field_index_offsets(input);
  }
  std::vector<PointCloudVector> radial_ordered_points;

  pcl::PointIndices no_ground_indices;

  if (elevation_grid_mode_) {
    convertPointcloudGridScan(input, radial_ordered_points);
    classifyPointCloudGridScan(input, radial_ordered_points, no_ground_indices);
  } else {
    convertPointcloud(input, radial_ordered_points);
    classifyPointCloud(input, radial_ordered_points, no_ground_indices);
  }
  output.row_step = no_ground_indices.indices.size() * input->point_step;
  output.data.resize(output.row_step);
  output.width = no_ground_indices.indices.size();
  output.fields = input->fields;
  output.is_dense = true;
  output.height = input->height;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.header = input->header;

  extractObjectPoints(input, no_ground_indices, output);
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

// TODO(taisa1): Temporary Implementation: Delete this function definition when all the filter
// nodes conform to new API.
void ScanGroundFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

rcl_interfaces::msg::SetParametersResult ScanGroundFilterComponent::onParameter(
  const std::vector<rclcpp::Parameter> & param)
{
  if (get_param(param, "grid_size_m", grid_size_m_)) {
    grid_mode_switch_grid_id_ = grid_mode_switch_radius_ / grid_size_m_;
    grid_size_rad_ =
      normalizeRadian(std::atan2(grid_mode_switch_radius_ + grid_size_m_, virtual_lidar_z_)) -
      normalizeRadian(std::atan2(grid_mode_switch_radius_, virtual_lidar_z_));
    tan_grid_size_rad_ = std::tan(grid_size_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting grid_size_m to: %f.", grid_size_m_);
    RCLCPP_DEBUG(
      get_logger(), "Setting grid_mode_switch_grid_id to: %f.", grid_mode_switch_grid_id_);
    RCLCPP_DEBUG(get_logger(), "Setting grid_size_rad to: %f.", grid_size_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting tan_grid_size_rad to: %f.", tan_grid_size_rad_);
  }
  if (get_param(param, "grid_mode_switch_radius", grid_mode_switch_radius_)) {
    grid_mode_switch_grid_id_ = grid_mode_switch_radius_ / grid_size_m_;
    grid_mode_switch_angle_rad_ = std::atan2(grid_mode_switch_radius_, virtual_lidar_z_);
    grid_size_rad_ =
      normalizeRadian(std::atan2(grid_mode_switch_radius_ + grid_size_m_, virtual_lidar_z_)) -
      normalizeRadian(std::atan2(grid_mode_switch_radius_, virtual_lidar_z_));
    tan_grid_size_rad_ = std::tan(grid_size_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting grid_mode_switch_radius to: %f.", grid_mode_switch_radius_);
    RCLCPP_DEBUG(
      get_logger(), "Setting grid_mode_switch_grid_id to: %f.", grid_mode_switch_grid_id_);
    RCLCPP_DEBUG(
      get_logger(), "Setting grid_mode_switch_angle_rad to: %f.", grid_mode_switch_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting grid_size_rad to: %f.", grid_size_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting tan_grid_size_rad to: %f.", tan_grid_size_rad_);
  }
  double global_slope_max_angle_deg{get_parameter("global_slope_max_angle_deg").as_double()};
  if (get_param(param, "global_slope_max_angle_deg", global_slope_max_angle_deg)) {
    global_slope_max_angle_rad_ = deg2rad(global_slope_max_angle_deg);
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      get_logger(), "Setting global_slope_max_angle_rad to: %f.", global_slope_max_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting global_slope_max_ratio to: %f.", global_slope_max_ratio_);
  }
  double local_slope_max_angle_deg{get_parameter("local_slope_max_angle_deg").as_double()};
  if (get_param(param, "local_slope_max_angle_deg", local_slope_max_angle_deg)) {
    local_slope_max_angle_rad_ = deg2rad(local_slope_max_angle_deg);
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      get_logger(), "Setting local_slope_max_angle_rad to: %f.", local_slope_max_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting local_slope_max_ratio to: %f.", local_slope_max_ratio_);
  }
  double radial_divider_angle_deg{get_parameter("radial_divider_angle_deg").as_double()};
  if (get_param(param, "radial_divider_angle_deg", radial_divider_angle_deg)) {
    radial_divider_angle_rad_ = deg2rad(radial_divider_angle_deg);
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    RCLCPP_DEBUG(
      get_logger(), "Setting radial_divider_angle_rad to: %f.", radial_divider_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting radial_dividers_num to: %zu.", radial_dividers_num_);
  }
  if (get_param(param, "split_points_distance_tolerance", split_points_distance_tolerance_)) {
    split_points_distance_tolerance_square_ =
      split_points_distance_tolerance_ * split_points_distance_tolerance_;
    RCLCPP_DEBUG(
      get_logger(), "Setting split_points_distance_tolerance to: %f.",
      split_points_distance_tolerance_);
    RCLCPP_DEBUG(
      get_logger(), "Setting split_points_distance_tolerance_square to: %f.",
      split_points_distance_tolerance_square_);
  }
  if (get_param(param, "split_height_distance", split_height_distance_)) {
    RCLCPP_DEBUG(get_logger(), "Setting split_height_distance to: %f.", split_height_distance_);
  }
  if (get_param(param, "use_virtual_ground_point", use_virtual_ground_point_)) {
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Setting use_virtual_ground_point to: " << std::boolalpha << use_virtual_ground_point_);
  }
  if (get_param(param, "use_recheck_ground_cluster", use_recheck_ground_cluster_)) {
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Setting use_recheck_ground_cluster to: " << std::boolalpha << use_recheck_ground_cluster_);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::ground_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_segmentation::ScanGroundFilterComponent)
