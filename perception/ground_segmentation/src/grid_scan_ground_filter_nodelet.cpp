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

#include "ground_segmentation/grid_scan_ground_filter_nodelet.hpp"

#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ground_segmentation
{
using pointcloud_preprocessor::get_param;
using tier4_autoware_utils::calcDistance3d;
using tier4_autoware_utils::deg2rad;
using tier4_autoware_utils::normalizeDegree;
using tier4_autoware_utils::normalizeRadian;
using vehicle_info_util::VehicleInfoUtil;

GridScanGroundFilterComponent::GridScanGroundFilterComponent(const rclcpp::NodeOptions & options)
: Filter("GridScanGroundFilter", options)
{
  // set initial parameters
  {
    detection_range_z_max_ = static_cast<float>(declare_parameter("detection_range_z_max", 2.5f));
    center_pcl_shift_ = static_cast<float>(declare_parameter("center_pcl_shift", 0.0));
    non_ground_height_threshold_ =
      static_cast<float>(declare_parameter("non_ground_height_threshold", 0.20));
    grid_mode_switch_radius_ =
      static_cast<float>(declare_parameter("grid_mode_switch_radius", 20.0));

    grid_size_m_ = static_cast<float>(declare_parameter("grid_size_m", 0.5));
    gnd_grid_buffer_size_ = static_cast<int>(declare_parameter("gnd_grid_buffer_size", 4));
    base_frame_ = declare_parameter("base_frame", "base_link");
    global_slope_max_angle_rad_ = deg2rad(declare_parameter("global_slope_max_angle_deg", 8.0));
    local_slope_max_angle_rad_ = deg2rad(declare_parameter("local_slope_max_angle_rad", 10.0));
    radial_divider_angle_rad_ = deg2rad(declare_parameter("radial_divider_angle_deg", 1.0));
    split_points_distance_tolerance_ = declare_parameter("split_points_distance_tolerance", 0.2);
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    vehicle_info_ = VehicleInfoUtil(*this).getVehicleInfo();

    grid_mode_switch_grid_id_ =
      grid_mode_switch_radius_ / grid_size_m_;  // changing the mode of grid division
    virtual_lidar_z_ = vehicle_info_.vehicle_height_m;
    grid_mode_switch_angle_rad_ = std::atan2(grid_mode_switch_radius_, virtual_lidar_z_);
    grid_size_rad_ = std::atan2(grid_mode_switch_radius_, virtual_lidar_z_) -
                     std::atan2(grid_mode_switch_radius_ - grid_size_m_, virtual_lidar_z_);
  }
  ground_pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/ground_pointcloud", rclcpp::SensorDataQoS());
  unknown_pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/unknown_pointcloud", rclcpp::SensorDataQoS());
  under_ground_pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/underground_pointcloud", rclcpp::SensorDataQoS());
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&GridScanGroundFilterComponent::onParameter, this, _1));
}

bool GridScanGroundFilterComponent::transformPointCloud(
  const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
  const PointCloud2::SharedPtr & out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(get_logger(), ex.what());
    return false;
  }
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

void GridScanGroundFilterComponent::convertPointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
  std::vector<PointCloudRefVector> & out_radial_ordered_points)
{
  out_radial_ordered_points.resize(radial_dividers_num_);
  PointRef current_point;
  uint16_t back_steps_num = 1;

  grid_size_rad_ =
    normalizeRadian(std::atan2(grid_mode_switch_radius_ + grid_size_m_, virtual_lidar_z_)) -
    normalizeRadian(std::atan2(grid_mode_switch_radius_, virtual_lidar_z_));
  for (size_t i = 0; i < in_cloud->points.size(); ++i) {
    auto x{
      in_cloud->points[i].x - vehicle_info_.wheel_base_m / 2.0f -
      center_pcl_shift_};  // base on front wheel center
    // auto y{in_cloud->points[i].y};
    auto radius{static_cast<float>(std::hypot(x, in_cloud->points[i].y))};
    auto theta{normalizeRadian(std::atan2(x, in_cloud->points[i].y), 0.0)};

    // divide by angle
    auto gama{normalizeRadian(std::atan2(radius, virtual_lidar_z_), 0.0f)};
    auto radial_div{
      static_cast<size_t>(std::floor(normalizeDegree(theta / radial_divider_angle_rad_, 0.0)))};
    uint16_t grid_id = grid_size_m_;
    float curr_grid_size = 0.0f;
    if (radius <= grid_mode_switch_radius_) {
      grid_id = static_cast<uint16_t>(radius / grid_size_m_);
      curr_grid_size = grid_size_m_;
    } else {
      grid_id = grid_mode_switch_grid_id_ + (gama - grid_mode_switch_angle_rad_) / grid_size_rad_;
      if (grid_id <= grid_mode_switch_grid_id_ + back_steps_num) {
        curr_grid_size = grid_size_m_;
      } else {
        curr_grid_size = (std::tan(gama) - std::tan(gama - grid_size_rad_)) * virtual_lidar_z_;
      }
    }
    current_point.grid_id = grid_id;
    current_point.grid_size = curr_grid_size;
    current_point.radius = radius;
    current_point.theta = theta;
    current_point.radial_div = radial_div;
    current_point.point_state = PointLabel::INIT;
    current_point.orig_index = i;
    current_point.orig_point = &in_cloud->points[i];

    // radial divisions
    out_radial_ordered_points[radial_div].emplace_back(current_point);
  }

  // sort by distance
  for (size_t i = 0; i < radial_dividers_num_; ++i) {
    std::sort(
      out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
      [](const PointRef & a, const PointRef & b) { return a.radius < b.radius; });
  }
}
void GridScanGroundFilterComponent::classifyPointCloud(
  std::vector<PointCloudRefVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices, pcl::PointIndices & out_ground_indices,
  pcl::PointIndices & out_unknown_indices, pcl::PointIndices & out_underground_indices)
{
  out_no_ground_indices.indices.clear();
  out_ground_indices.indices.clear();
  out_unknown_indices.indices.clear();
  out_underground_indices.indices.clear();
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) {
    PointsCentroid ground_cluster, non_ground_cluster;
    ground_cluster.initialize();
    non_ground_cluster.initialize();
    std::vector<float> gnd_mean_z_list;
    std::vector<float> gnd_radius_list;
    std::vector<uint16_t> gnd_grid_id_list;
    std::vector<float> gnd_max_z_list;

    // check empty ray:
    if (in_radial_ordered_clouds[i].size() == 0) {
      continue;
    }

    // check the first point in ray
    auto * p = &in_radial_ordered_clouds[i][0];
    PointRef * prev_p;
    prev_p = &in_radial_ordered_clouds[i][0];  // for checking the distance to prev point

    bool initilized_flg = false;
    bool prev_list_initilize = false;

    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) {
      p = &in_radial_ordered_clouds[i][j];

      float global_slope_p = 0.0f;
      // float local_slope_curr_p = 0.0f;
      global_slope_p = std::atan(p->orig_point->z / p->radius);
      // chec if the current point cloud is new grid
      float non_ground_height_threshold_adap = non_ground_height_threshold_;
      if (p->orig_point->x < -less_interest_dist_) {
        non_ground_height_threshold_adap =
          non_ground_height_threshold_ * (p->radius / less_interest_dist_);
      }
      if ((initilized_flg == false) && (p->radius <= first_ring_distance_)) {
        // add condition for suddent slope, but it lose ability to detect 20cm object near by
        if (
          (global_slope_p >= global_slope_max_angle_rad_) &&
          p->orig_point->z > non_ground_height_threshold_adap) {
          out_no_ground_indices.indices.push_back(p->orig_index);
        } else if (
          (abs(global_slope_p) < global_slope_max_angle_rad_) &&
          abs(p->orig_point->z) < non_ground_height_threshold_adap) {
          out_ground_indices.indices.push_back(p->orig_index);
          ground_cluster.addPoint(p->radius, p->orig_point->z);
          if (p->grid_id > prev_p->grid_id) {
            initilized_flg = true;
          }
        }
      } else {
        if (prev_list_initilize == false) {
          if (initilized_flg) {
            // first grid is gnd:
            // initilize prev list by first gnd grid:
            for (int ind_grid = p->grid_id - 1 - gnd_grid_buffer_size_; ind_grid < p->grid_id - 1;
                 ind_grid++) {
              gnd_mean_z_list.push_back(
                (ind_grid - p->grid_id + 1 + gnd_grid_buffer_size_) *
                ground_cluster.getAverageHeight() / static_cast<float>(gnd_grid_buffer_size_));
              gnd_radius_list.push_back(
                (ind_grid - p->grid_id + 1 + gnd_grid_buffer_size_) *
                ground_cluster.getAverageRadius() / static_cast<float>(gnd_grid_buffer_size_));
              gnd_grid_id_list.push_back(ind_grid);
              gnd_max_z_list.push_back(
                static_cast<float>(ind_grid) * ground_cluster.getMaxheight() /
                static_cast<float>(gnd_grid_buffer_size_));
            }

          } else {
            // assume first gnd grid is zero
            for (int ind_grid = p->grid_id - 1 - gnd_grid_buffer_size_; ind_grid < p->grid_id;
                 ind_grid++) {
              gnd_mean_z_list.push_back(0.0f);
              gnd_radius_list.push_back(p->radius - ind_grid * grid_size_m_);
              gnd_grid_id_list.push_back(ind_grid);
              gnd_max_z_list.push_back(0.0f);
            }
          }
          prev_list_initilize = true;
        }
        if (p->grid_id > prev_p->grid_id) {
          // check if the prev grid have ground point cloud:
          if (ground_cluster.getAverageRadius() > 0.0) {
            gnd_mean_z_list.push_back(ground_cluster.getAverageHeight());
            gnd_max_z_list.push_back(ground_cluster.getMaxheight());
            gnd_grid_id_list.push_back(prev_p->grid_id);
            gnd_radius_list.push_back(ground_cluster.getAverageRadius());
            ground_cluster.initialize();
          }
        }
        if (p->orig_point->z - gnd_mean_z_list.back() > detection_range_z_max_) {
          out_unknown_indices.indices.push_back(p->orig_index);
          p->point_state = PointLabel::OUT_OF_RANGE;
        } else if (
          prev_p->point_state == PointLabel::NON_GROUND &&
          std::hypot(
            p->orig_point->x - prev_p->orig_point->x, p->orig_point->y - prev_p->orig_point->y) <
            split_points_distance_tolerance_ &&
          p->orig_point->z > prev_p->orig_point->z) {
          p->point_state = PointLabel::NON_GROUND;
          out_no_ground_indices.indices.push_back(p->orig_index);
        } else {
          float next_gnd_z = 0.0f;
          float curr_gnd_slope = 0.0f;
          float gnd_buffer_z_mean = 0.0f;
          float gnd_buffer_radius = 0.0f;
          float gnd_buffer_z_max = 0.0f;
          for (int i_ref = gnd_grid_buffer_size_ + 1; i_ref > 1; i_ref--) {
            gnd_buffer_z_mean += *(gnd_mean_z_list.end() - i_ref);
            gnd_buffer_radius += *(gnd_radius_list.end() - i_ref);
            gnd_buffer_z_max += *(gnd_max_z_list.end() - i_ref);
          }
          gnd_buffer_z_mean /= static_cast<float>(gnd_grid_buffer_size_ - 1);
          gnd_buffer_radius /= static_cast<float>(gnd_grid_buffer_size_ - 1);
          gnd_buffer_z_max /= static_cast<float>(gnd_grid_buffer_size_ - 1);
          curr_gnd_slope = std::atan(
            (gnd_mean_z_list.back() - gnd_buffer_z_mean) /
            (gnd_radius_list.back() - gnd_buffer_radius));
          curr_gnd_slope = curr_gnd_slope < -global_slope_max_angle_rad_
                             ? -global_slope_max_angle_rad_
                             : curr_gnd_slope;
          curr_gnd_slope = curr_gnd_slope > global_slope_max_angle_rad_
                             ? global_slope_max_angle_rad_
                             : curr_gnd_slope;

          next_gnd_z =
            std::tan(curr_gnd_slope) * (p->radius - gnd_buffer_radius) + gnd_buffer_z_mean;
          float gnd_z_threshold = std::tan(DEG2RAD(5.0f)) * (p->radius - gnd_radius_list.back());
          float local_slope_p = std::atan(
            (p->orig_point->z - *(gnd_mean_z_list.end() - 2)) /
            (p->radius - *(gnd_radius_list.end() - 2)));

          if (global_slope_p > global_slope_max_angle_rad_) {
            out_no_ground_indices.indices.push_back(p->orig_index);
          } else {
            if (
              (p->grid_id <
               *(gnd_grid_id_list.end() - gnd_grid_buffer_size_) + gnd_grid_buffer_size_ + 3) &&
              (p->radius - gnd_radius_list.back() < 3 * p->grid_size)) {
              if (((abs(p->orig_point->z - next_gnd_z) <=
                    non_ground_height_threshold_adap + gnd_z_threshold) ||
                   (abs(local_slope_p) < local_slope_max_angle_rad_))) {
                out_ground_indices.indices.push_back(p->orig_index);
                // if (abs(p->orig_point->z - next_gnd_z) < gnd_z_threshold) {
                ground_cluster.addPoint(p->radius, p->orig_point->z);
                p->point_state = PointLabel::GROUND;
                // }
              } else if (
                p->orig_point->z - next_gnd_z >
                non_ground_height_threshold_adap + gnd_z_threshold) {
                out_no_ground_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::NON_GROUND;
              } else if (
                p->orig_point->z - next_gnd_z <
                -(non_ground_height_threshold_adap + gnd_z_threshold)) {
                out_underground_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::OUT_OF_RANGE;
              } else {
                out_unknown_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::UNKNOWN;
              }

            } else if (
              (p->radius - gnd_radius_list.back() < 3 * p->grid_size) ||
              (p->radius < grid_mode_switch_radius_ * 2.0f)) {
              local_slope_p = std::atan(
                (p->orig_point->z - gnd_mean_z_list.back()) / (p->radius - gnd_radius_list.back()));

              if (
                (abs(local_slope_p) < local_slope_max_angle_rad_) ||
                (abs(p->orig_point->z - gnd_mean_z_list.back()) <
                 non_ground_height_threshold_adap) ||
                (abs(p->orig_point->z - gnd_max_z_list.back()) <
                 non_ground_height_threshold_adap)) {
                out_ground_indices.indices.push_back(p->orig_index);
                ground_cluster.addPoint(p->radius, p->orig_point->z);
                p->point_state = PointLabel::GROUND;
              } else if (local_slope_p > global_slope_max_angle_rad_) {
                out_no_ground_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::NON_GROUND;
              } else if (local_slope_p < -global_slope_max_angle_rad_) {
                out_underground_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::OUT_OF_RANGE;
              } else {
                out_unknown_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::UNKNOWN;
              }
            } else {
              // checking by reference only the last gnd grid
              local_slope_p = std::atan(
                (p->orig_point->z - gnd_mean_z_list.back()) / (p->radius - gnd_radius_list.back()));

              if ((abs(local_slope_p) < global_slope_max_angle_rad_)) {
                out_ground_indices.indices.push_back(p->orig_index);
                ground_cluster.addPoint(p->radius, p->orig_point->z);
                p->point_state = PointLabel::GROUND;
              } else if (local_slope_p > global_slope_max_angle_rad_) {
                out_no_ground_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::NON_GROUND;
              } else if (local_slope_p < -global_slope_max_angle_rad_) {
                out_underground_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::OUT_OF_RANGE;
              } else {
                out_unknown_indices.indices.push_back(p->orig_index);
                p->point_state = PointLabel::UNKNOWN;
              }
            }
          }
          // }
        }
      }
      prev_p = p;
    }
    // estimate the height from predicted current ground and compare with threshold

    // estimate the local slope to previous virtual gnd and compare with
  }
}

void GridScanGroundFilterComponent::extractObjectPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_object_cloud_ptr)
{
  for (const auto & i : in_indices.indices) {
    out_object_cloud_ptr->points.emplace_back(in_cloud_ptr->points[i]);
  }
}

void GridScanGroundFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  auto input_transformed_ptr = std::make_shared<PointCloud2>();
  bool succeeded = transformPointCloud(base_frame_, input, input_transformed_ptr);
  sensor_frame_ = input->header.frame_id;
  if (!succeeded) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Failed transform from " << base_frame_ << " to " << input->header.frame_id);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_transformed_ptr, *current_sensor_cloud_ptr);

  std::vector<PointCloudRefVector> radial_ordered_points;

  convertPointcloud(current_sensor_cloud_ptr, radial_ordered_points);

  pcl::PointIndices no_ground_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  no_ground_cloud_ptr->points.reserve(current_sensor_cloud_ptr->points.size());

  pcl::PointIndices ground_pcl_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  ground_cloud_ptr->points.reserve(current_sensor_cloud_ptr->points.size());

  pcl::PointIndices unknown_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr unknown_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  unknown_cloud_ptr->points.reserve(current_sensor_cloud_ptr->points.size());

  pcl::PointIndices underground_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr underground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  underground_cloud_ptr->points.reserve(current_sensor_cloud_ptr->points.size());

  // classifyPointCloud(radial_ordered_points, no_ground_indices);
  classifyPointCloud(
    radial_ordered_points, no_ground_indices, ground_pcl_indices, unknown_indices,
    underground_indices);

  extractObjectPoints(current_sensor_cloud_ptr, no_ground_indices, no_ground_cloud_ptr);
  extractObjectPoints(current_sensor_cloud_ptr, ground_pcl_indices, ground_cloud_ptr);
  extractObjectPoints(current_sensor_cloud_ptr, unknown_indices, unknown_cloud_ptr);
  extractObjectPoints(current_sensor_cloud_ptr, underground_indices, underground_cloud_ptr);

  sensor_msgs::msg::PointCloud2 ground_pcl_msg;
  pcl::toROSMsg(*ground_cloud_ptr, ground_pcl_msg);
  ground_pcl_msg.header = input->header;
  ground_pcl_pub_->publish(ground_pcl_msg);

  sensor_msgs::msg::PointCloud2 unknown_pcl_msg;
  pcl::toROSMsg(*unknown_cloud_ptr, unknown_pcl_msg);
  unknown_pcl_msg.header = input->header;
  unknown_pcl_pub_->publish(unknown_pcl_msg);

  sensor_msgs::msg::PointCloud2 underground_msg;
  pcl::toROSMsg(*underground_cloud_ptr, underground_msg);
  underground_msg.header = input->header;
  under_ground_pcl_pub_->publish(underground_msg);

  auto no_ground_cloud_msg_ptr = std::make_shared<PointCloud2>();
  pcl::toROSMsg(*no_ground_cloud_ptr, *no_ground_cloud_msg_ptr);

  no_ground_cloud_msg_ptr->header.stamp = input->header.stamp;
  no_ground_cloud_msg_ptr->header.frame_id = base_frame_;
  output = *no_ground_cloud_msg_ptr;
}

rcl_interfaces::msg::SetParametersResult GridScanGroundFilterComponent::onParameter(
  const std::vector<rclcpp::Parameter> & p)
{
  if (get_param(p, "base_frame", base_frame_)) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Setting base_frame to: " << base_frame_);
  }
  double global_slope_max_angle_deg{get_parameter("global_slope_max_angle_deg").as_double()};
  if (get_param(p, "global_slope_max_angle_deg", global_slope_max_angle_deg)) {
    global_slope_max_angle_rad_ = deg2rad(global_slope_max_angle_deg);
    RCLCPP_DEBUG(
      get_logger(), "Setting global_slope_max_angle_rad to: %f.", global_slope_max_angle_rad_);
  }
  double local_slope_max_angle_deg{get_parameter("local_slope_max_angle_deg").as_double()};
  if (get_param(p, "local_slope_max_angle_deg", local_slope_max_angle_deg)) {
    local_slope_max_angle_rad_ = deg2rad(local_slope_max_angle_deg);
    RCLCPP_DEBUG(
      get_logger(), "Setting local_slope_max_angle_rad to: %f.", local_slope_max_angle_rad_);
  }
  double radial_divider_angle_deg{get_parameter("radial_divider_angle_deg").as_double()};
  if (get_param(p, "radial_divider_angle_deg", radial_divider_angle_deg)) {
    radial_divider_angle_rad_ = deg2rad(radial_divider_angle_deg);
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    RCLCPP_DEBUG(
      get_logger(), "Setting radial_divider_angle_rad to: %f.", radial_divider_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting radial_dividers_num to: %zu.", radial_dividers_num_);
  }
  if (get_param(p, "split_points_distance_tolerance", split_points_distance_tolerance_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting split_points_distance_tolerance to: %f.",
      split_points_distance_tolerance_);
  }
  if (get_param(p, "detection_range_z_max", detection_range_z_max_)) {
    RCLCPP_DEBUG(get_logger(), "Setting detection_range_z_max to: %f.", detection_range_z_max_);
  }
  if (get_param(p, "center_pcl_shift", center_pcl_shift_)) {
    RCLCPP_DEBUG(get_logger(), "Setting center_pcl_shift to: %f.", center_pcl_shift_);
  }
  if (get_param(p, "non_ground_height_threshold", non_ground_height_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting non_ground_height_threshold to: %f.", non_ground_height_threshold_);
  }
  if (get_param(p, "grid_size_m", grid_size_m_)) {
    RCLCPP_DEBUG(get_logger(), "Setting grid_size_m to: %f.", grid_size_m_);
  }
  if (get_param(p, "gnd_grid_buffer_size", gnd_grid_buffer_size_)) {
    RCLCPP_DEBUG(get_logger(), "Setting gnd_grid_buffer_size to: %d.", gnd_grid_buffer_size_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace ground_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ground_segmentation::GridScanGroundFilterComponent)
