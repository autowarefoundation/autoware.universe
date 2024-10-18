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

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

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
    // modes
    elevation_grid_mode_ = declare_parameter<bool>("elevation_grid_mode");

    // common parameters
    radial_divider_angle_rad_ =
      static_cast<float>(deg2rad(declare_parameter<double>("radial_divider_angle_deg")));
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);

    // common thresholds
    global_slope_max_angle_rad_ =
      static_cast<float>(deg2rad(declare_parameter<double>("global_slope_max_angle_deg")));
    local_slope_max_angle_rad_ =
      static_cast<float>(deg2rad(declare_parameter<double>("local_slope_max_angle_deg")));
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    split_points_distance_tolerance_ =
      static_cast<float>(declare_parameter<double>("split_points_distance_tolerance"));
    split_points_distance_tolerance_square_ =
      split_points_distance_tolerance_ * split_points_distance_tolerance_;

    // vehicle info
    vehicle_info_ = VehicleInfoUtils(*this).getVehicleInfo();

    // non-grid parameters
    use_virtual_ground_point_ = declare_parameter<bool>("use_virtual_ground_point");
    split_height_distance_ = static_cast<float>(declare_parameter<double>("split_height_distance"));

    // grid mode parameters
    use_recheck_ground_cluster_ = declare_parameter<bool>("use_recheck_ground_cluster");
    use_lowest_point_ = declare_parameter<bool>("use_lowest_point");
    detection_range_z_max_ = static_cast<float>(declare_parameter<double>("detection_range_z_max"));
    low_priority_region_x_ = static_cast<float>(declare_parameter<double>("low_priority_region_x"));
    center_pcl_shift_ = static_cast<float>(declare_parameter<double>("center_pcl_shift"));
    non_ground_height_threshold_ =
      static_cast<float>(declare_parameter<double>("non_ground_height_threshold"));

    // grid parameters
    grid_size_m_ = static_cast<float>(declare_parameter<double>("grid_size_m"));
    grid_mode_switch_radius_ =
      static_cast<float>(declare_parameter<double>("grid_mode_switch_radius"));
    gnd_grid_buffer_size_ = declare_parameter<int>("gnd_grid_buffer_size");
    virtual_lidar_z_ = vehicle_info_.vehicle_height_m;

    // initialize grid
    grid_.initialize(grid_size_m_, grid_mode_switch_radius_, virtual_lidar_z_);

    // data access
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
  const PointCloud2ConstPtr & input, const size_t data_index, pcl::PointXYZ & point) const
{
  point.x = *reinterpret_cast<const float *>(&input->data[data_index + data_offset_x_]);
  point.y = *reinterpret_cast<const float *>(&input->data[data_index + data_offset_y_]);
  point.z = *reinterpret_cast<const float *>(&input->data[data_index + data_offset_z_]);
}

void ScanGroundFilterComponent::convertPointcloudGridScan(
  const PointCloud2ConstPtr & in_cloud,
  std::vector<PointCloudVector> & out_radial_ordered_points) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  out_radial_ordered_points.resize(radial_dividers_num_);
  PointData current_point;

  const auto inv_radial_divider_angle_rad = 1.0f / radial_divider_angle_rad_;
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
      // Get Point
      get_point_from_data_index(in_cloud, data_index, input_point);

      // determine the azimuth angle group
      auto x{input_point.x - x_shift};  // base on front wheel center
      auto radius{static_cast<float>(std::hypot(x, input_point.y))};
      auto theta{normalizeRadian(std::atan2(x, input_point.y), 0.0)};
      auto radial_div{static_cast<size_t>(std::floor(theta * inv_radial_divider_angle_rad))};

      current_point.radius = radius;
      current_point.point_state = PointLabel::INIT;
      current_point.orig_index = point_index;
      current_point.grid_id = grid_.getGridId(radius);

      // store the point in the corresponding radial division
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
  const PointCloud2ConstPtr & in_cloud,
  std::vector<PointCloudVector> & out_radial_ordered_points) const
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
      // Get Point
      get_point_from_data_index(in_cloud, data_index, input_point);

      // determine the azimuth angle group
      auto radius{static_cast<float>(std::hypot(input_point.x, input_point.y))};
      auto theta{normalizeRadian(std::atan2(input_point.x, input_point.y), 0.0)};
      auto radial_div{static_cast<size_t>(std::floor(theta * inv_radial_divider_angle_rad))};

      current_point.radius = radius;
      current_point.point_state = PointLabel::INIT;
      current_point.orig_index = point_index;

      // store the point in the corresponding radial division
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

void ScanGroundFilterComponent::calcVirtualGroundOrigin(pcl::PointXYZ & point) const
{
  point.x = vehicle_info_.wheel_base_m;
  point.y = 0;
  point.z = 0;
}

void ScanGroundFilterComponent::initializeFirstGndGrids(
  const float h, const float r, const uint16_t id, std::vector<GridCenter> & gnd_grids) const
{
  // initialize gnd_grids
  gnd_grids.clear();
  gnd_grids.reserve(gnd_grid_buffer_size_);

  // Index of grids
  //   id is the first grid, will be filled by initial centroid_bin
  //   id - gnd_grid_buffer_size_ is the first grid of zero-zero position
  //   the intermediate grids are generated by linear interpolation
  const uint16_t estimating_grid_num = gnd_grid_buffer_size_ + 1;
  const uint16_t idx_estimate_from = id - estimating_grid_num;
  const float gradient = h / r;

  GridCenter curr_gnd_grid;
  for (uint16_t idx = 1; idx < estimating_grid_num; ++idx) {
    float interpolation_ratio = static_cast<float>(idx) / static_cast<float>(estimating_grid_num);
    const uint16_t ind_grid = idx_estimate_from + idx;

    const float interpolated_r = r * interpolation_ratio;
    const float interpolated_z = gradient * interpolated_r;

    curr_gnd_grid.radius = interpolated_r;
    curr_gnd_grid.avg_height = interpolated_z;
    curr_gnd_grid.max_height = interpolated_z;
    curr_gnd_grid.gradient = gradient;
    curr_gnd_grid.intercept = 0.0f;
    curr_gnd_grid.grid_id = ind_grid;
    gnd_grids.push_back(curr_gnd_grid);
  }
}

void ScanGroundFilterComponent::fitLineFromGndGrid(
  const std::vector<GridCenter> & gnd_grids_list, const size_t start_idx, const size_t end_idx,
  float & a, float & b) const
{
  // calculate local gradient by least square method
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xy = 0.0f;
  float sum_x2 = 0.0f;
  for (auto it = gnd_grids_list.begin() + start_idx; it < gnd_grids_list.begin() + end_idx; ++it) {
    sum_x += it->radius;
    sum_y += it->avg_height;
    sum_xy += it->radius * it->avg_height;
    sum_x2 += it->radius * it->radius;
  }
  const float n = static_cast<float>(end_idx - start_idx);

  // y = a * x + b
  a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
  b = (sum_y - a * sum_x) / n;
}

void ScanGroundFilterComponent::checkContinuousGndGrid(
  PointData & pd, const pcl::PointXYZ & point_curr,
  const std::vector<GridCenter> & gnd_grids_list) const
{
  // 1. check local slope
  {
    // reference gird is the last-1
    const auto reference_grid_it = gnd_grids_list.end() - 2;
    const float delta_z = point_curr.z - reference_grid_it->avg_height;
    const float delta_radius = pd.radius - reference_grid_it->radius;
    const float local_slope_ratio = delta_z / delta_radius;

    if (abs(local_slope_ratio) < local_slope_max_ratio_) {
      pd.point_state = PointLabel::GROUND;
      return;
    }
  }

  // 2. mean of grid buffer(filtering)
  const float gradient =
    std::clamp(gnd_grids_list.back().gradient, -global_slope_max_ratio_, global_slope_max_ratio_);
  const float & intercept = gnd_grids_list.back().intercept;

  // extrapolate next ground height
  const float next_gnd_z = gradient * pd.radius + intercept;

  // calculate fixed angular threshold from the reference position
  const float gnd_z_local_thresh =
    std::tan(DEG2RAD(5.0)) * (pd.radius - gnd_grids_list.back().radius);

  if (abs(point_curr.z - next_gnd_z) <= non_ground_height_threshold_ + gnd_z_local_thresh) {
    pd.point_state = PointLabel::GROUND;
    return;
  }
  if (point_curr.z - next_gnd_z >= non_ground_height_threshold_ + gnd_z_local_thresh) {
    pd.point_state = PointLabel::NON_GROUND;
    return;
  }
}

void ScanGroundFilterComponent::checkDiscontinuousGndGrid(
  PointData & pd, const pcl::PointXYZ & point_curr,
  const std::vector<GridCenter> & gnd_grids_list) const
{
  const auto & grid_ref = gnd_grids_list.back();
  const float delta_avg_z = point_curr.z - grid_ref.avg_height;
  if (abs(delta_avg_z) < non_ground_height_threshold_) {
    pd.point_state = PointLabel::GROUND;
    return;
  }
  const float delta_max_z = point_curr.z - grid_ref.max_height;
  if (abs(delta_max_z) < non_ground_height_threshold_) {
    pd.point_state = PointLabel::GROUND;
    return;
  }
  const float delta_radius = pd.radius - grid_ref.radius;
  const float local_slope_ratio = delta_avg_z / delta_radius;
  if (abs(local_slope_ratio) < local_slope_max_ratio_) {
    pd.point_state = PointLabel::GROUND;
    return;
  }
  if (local_slope_ratio >= local_slope_max_ratio_) {
    pd.point_state = PointLabel::NON_GROUND;
    return;
  }
}

void ScanGroundFilterComponent::checkBreakGndGrid(
  PointData & pd, const pcl::PointXYZ & point_curr,
  const std::vector<GridCenter> & gnd_grids_list) const
{
  const auto & grid_ref = gnd_grids_list.back();
  const float delta_avg_z = point_curr.z - grid_ref.avg_height;
  const float delta_radius = pd.radius - grid_ref.radius;
  const float local_slope_ratio = delta_avg_z / delta_radius;
  if (abs(local_slope_ratio) < global_slope_max_ratio_) {
    pd.point_state = PointLabel::GROUND;
    return;
  }
  if (local_slope_ratio >= global_slope_max_ratio_) {
    pd.point_state = PointLabel::NON_GROUND;
    return;
  }
}

void ScanGroundFilterComponent::recheckGroundCluster(
  const PointsCentroid & gnd_cluster, const float non_ground_threshold, const bool use_lowest_point,
  pcl::PointIndices & non_ground_indices) const
{
  const float reference_height =
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
  pcl::PointIndices & out_no_ground_indices) const
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

    PointData pd_curr, pd_prev;
    pcl::PointXYZ point_curr, point_prev;

    // initialize the previous point
    {
      pd_curr = in_radial_ordered_clouds[i][0];
      const size_t data_index = in_cloud->point_step * pd_curr.orig_index;
      get_point_from_data_index(in_cloud, data_index, point_curr);
    }

    // iterate over the points in the ray
    for (const auto & point : in_radial_ordered_clouds[i]) {
      // set the previous point
      pd_prev = pd_curr;
      point_prev = point_curr;

      // set the current point
      pd_curr = point;
      const size_t data_index = in_cloud->point_step * pd_curr.orig_index;
      get_point_from_data_index(in_cloud, data_index, point_curr);

      // determine if the current point is in new grid
      const bool is_curr_in_next_grid = pd_curr.grid_id > pd_prev.grid_id;

      // initialization process for the first grid and interpolate the previous grids
      if (!initialized_first_gnd_grid) {
        // set the thresholds
        const float global_slope_ratio_p = point_prev.z / pd_prev.radius;
        float non_ground_height_threshold_local = non_ground_height_threshold_;
        if (point_prev.x < low_priority_region_x_) {
          non_ground_height_threshold_local =
            non_ground_height_threshold_ * abs(point_prev.x / low_priority_region_x_);
        }
        // non_ground_height_threshold_local is only for initialization

        // prepare centroid_bin for the first grid
        if (
          // classify previous point
          global_slope_ratio_p >= global_slope_max_ratio_ &&
          point_prev.z > non_ground_height_threshold_local) {
          out_no_ground_indices.indices.push_back(pd_prev.orig_index);
          pd_prev.point_state = PointLabel::NON_GROUND;
        } else if (
          abs(global_slope_ratio_p) < global_slope_max_ratio_ &&
          abs(point_prev.z) < non_ground_height_threshold_local) {
          centroid_bin.addPoint(pd_prev.radius, point_prev.z, pd_prev.orig_index);
          pd_prev.point_state = PointLabel::GROUND;
          // centroid_bin is filled at least once
          // if the current point is in the next gird, it is ready to be initialized
          initialized_first_gnd_grid = is_curr_in_next_grid;
        }
        // keep filling the centroid_bin until it is ready to be initialized
        if (!initialized_first_gnd_grid) {
          continue;
        }
        // estimate previous grids by linear interpolation
        float h = centroid_bin.getAverageHeight();
        float r = centroid_bin.getAverageRadius();
        initializeFirstGndGrids(h, r, pd_prev.grid_id, gnd_grids);
      }

      // finalize the current centroid_bin and update the gnd_grids
      if (is_curr_in_next_grid && centroid_bin.getIndicesRef().indices.size() > 0) {
        // check if the prev grid have ground point cloud
        if (use_recheck_ground_cluster_) {
          recheckGroundCluster(
            centroid_bin, non_ground_height_threshold_, use_lowest_point_, out_no_ground_indices);
          // centroid_bin is not modified. should be rechecked by out_no_ground_indices?
        }
        // convert the centroid_bin to grid-center and add it to the gnd_grids
        GridCenter curr_gnd_grid;
        curr_gnd_grid.radius = centroid_bin.getAverageRadius();
        curr_gnd_grid.avg_height = centroid_bin.getAverageHeight();
        curr_gnd_grid.max_height = centroid_bin.getMaxHeight();
        curr_gnd_grid.grid_id = pd_prev.grid_id;
        curr_gnd_grid.gradient = 0.0f;   // not calculated yet
        curr_gnd_grid.intercept = 0.0f;  // not calculated yet
        gnd_grids.push_back(curr_gnd_grid);
        // clear the centroid_bin
        centroid_bin.initialize();

        // calculate local ground gradient
        float gradient, intercept;
        fitLineFromGndGrid(
          gnd_grids, gnd_grids.size() - gnd_grid_buffer_size_, gnd_grids.size(), gradient,
          intercept);
        // update the current grid
        gnd_grids.back().gradient = gradient;    // update the gradient
        gnd_grids.back().intercept = intercept;  // update the intercept
      }

      // 0: set the thresholds
      const float global_slope_ratio_p = point_curr.z / pd_curr.radius;
      const auto & grid_ref = gnd_grids.back();

      // 1: height is out-of-range
      if (point_curr.z - grid_ref.avg_height > detection_range_z_max_) {
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

      const uint16_t next_gnd_grid_id_thresh = (gnd_grids.end() - gnd_grid_buffer_size_)->grid_id +
                                               gnd_grid_buffer_size_ + gnd_grid_continual_thresh_;
      const float curr_grid_width = grid_.getGridSize(pd_curr.radius, pd_curr.grid_id);
      if (
        // 4: the point is continuous with the previous grid
        pd_curr.grid_id < next_gnd_grid_id_thresh &&
        pd_curr.radius - grid_ref.radius < gnd_grid_continual_thresh_ * curr_grid_width) {
        checkContinuousGndGrid(pd_curr, point_curr, gnd_grids);
      } else if (
        // 5: the point is discontinuous with the previous grid
        pd_curr.radius - grid_ref.radius < gnd_grid_continual_thresh_ * curr_grid_width) {
        checkDiscontinuousGndGrid(pd_curr, point_curr, gnd_grids);
      } else {
        // 6: the point is break the previous grid
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
  const PointCloud2ConstPtr & in_cloud,
  const std::vector<PointCloudVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices) const
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
    PointLabel point_label_curr = PointLabel::INIT;

    pcl::PointXYZ prev_gnd_point(0, 0, 0), point_curr, point_prev;

    // iterate over the points in the ray
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); ++j) {
      float points_distance = 0.0f;
      const float local_slope_max_angle = local_slope_max_angle_rad_;

      // set the previous point
      point_prev = point_curr;
      PointLabel point_label_prev = point_label_curr;

      // set the current point
      const PointData & pd = in_radial_ordered_clouds[i][j];
      point_label_curr = pd.point_state;

      const size_t data_index = in_cloud->point_step * pd.orig_index;
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

      float radius_distance_from_gnd = pd.radius - prev_gnd_radius;
      float height_from_gnd = point_curr.z - prev_gnd_point.z;
      float height_from_obj = point_curr.z - non_ground_cluster.getAverageHeight();
      bool calculate_slope = false;
      bool is_point_close_to_prev =
        (points_distance <
         (pd.radius * radial_divider_angle_rad_ + split_points_distance_tolerance_));

      float global_slope_ratio = point_curr.z / pd.radius;
      // check points which is far enough from previous point
      if (global_slope_ratio > global_slope_max_ratio_) {
        point_label_curr = PointLabel::NON_GROUND;
        calculate_slope = false;
      } else if (
        (point_label_prev == PointLabel::NON_GROUND) &&
        (std::abs(height_from_obj) >= split_height_distance_)) {
        calculate_slope = true;
      } else if (is_point_close_to_prev && std::abs(height_from_gnd) < split_height_distance_) {
        // close to the previous point, set point follow label
        point_label_curr = PointLabel::POINT_FOLLOW;
        calculate_slope = false;
      } else {
        calculate_slope = true;
      }
      if (is_point_close_to_prev) {
        height_from_gnd = point_curr.z - ground_cluster.getAverageHeight();
        radius_distance_from_gnd = pd.radius - ground_cluster.getAverageRadius();
      }
      if (calculate_slope) {
        // far from the previous point
        auto local_slope = std::atan2(height_from_gnd, radius_distance_from_gnd);
        if (local_slope - prev_gnd_slope > local_slope_max_angle) {
          // the point is outside of the local slope threshold
          point_label_curr = PointLabel::NON_GROUND;
        } else {
          point_label_curr = PointLabel::GROUND;
        }
      }

      if (point_label_curr == PointLabel::GROUND) {
        ground_cluster.initialize();
        non_ground_cluster.initialize();
      }
      if (point_label_curr == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(pd.orig_index);
      } else if (  // NOLINT
        (point_label_prev == PointLabel::NON_GROUND) &&
        (point_label_curr == PointLabel::POINT_FOLLOW)) {
        point_label_curr = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(pd.orig_index);
      } else if (  // NOLINT
        (point_label_prev == PointLabel::GROUND) &&
        (point_label_curr == PointLabel::POINT_FOLLOW)) {
        point_label_curr = PointLabel::GROUND;
      } else {
      }

      // update the ground state
      if (point_label_curr == PointLabel::GROUND) {
        prev_gnd_radius = pd.radius;
        prev_gnd_point = pcl::PointXYZ(point_curr.x, point_curr.y, point_curr.z);
        ground_cluster.addPoint(pd.radius, point_curr.z);
        prev_gnd_slope = ground_cluster.getAverageSlope();
      }
      // update the non ground state
      if (point_label_curr == PointLabel::NON_GROUND) {
        non_ground_cluster.addPoint(pd.radius, point_curr.z);
      }
    }
  }
}

void ScanGroundFilterComponent::extractObjectPoints(
  const PointCloud2ConstPtr & in_cloud_ptr, const pcl::PointIndices & in_indices,
  PointCloud2 & out_object_cloud) const
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
    grid_.initialize(grid_size_m_, grid_mode_switch_radius_, virtual_lidar_z_);
  }
  if (get_param(param, "grid_mode_switch_radius", grid_mode_switch_radius_)) {
    grid_.initialize(grid_size_m_, grid_mode_switch_radius_, virtual_lidar_z_);
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
