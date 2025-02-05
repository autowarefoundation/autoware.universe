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

#include "autoware/pointcloud_preprocessor/outlier_filter/dual_return_outlier_filter_node.hpp"

#include "autoware/point_types/types.hpp"

#include <std_msgs/msg/header.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRCAEDT;
using autoware::point_types::ReturnType;
using diagnostic_msgs::msg::DiagnosticStatus;

DualReturnOutlierFilterComponent::DualReturnOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("DualReturnOutlierFilter", options)
{
  // set initial parameters
  {
    x_max_ = declare_parameter<float>("x_max");
    x_min_ = declare_parameter<float>("x_min");
    y_max_ = declare_parameter<float>("y_max");
    y_min_ = declare_parameter<float>("y_min");
    z_max_ = declare_parameter<float>("z_max");
    z_min_ = declare_parameter<float>("z_min");
    min_azimuth_deg_ = declare_parameter<float>("min_azimuth_deg");
    max_azimuth_deg_ = declare_parameter<float>("max_azimuth_deg");
    max_distance_ = declare_parameter<float>("max_distance");
    vertical_bins_ = declare_parameter<int>("vertical_bins");
    max_azimuth_diff_ = declare_parameter<float>("max_azimuth_diff");
    weak_first_distance_ratio_ = declare_parameter<double>("weak_first_distance_ratio");
    general_distance_ratio_ = declare_parameter<double>("general_distance_ratio");

    weak_first_local_noise_threshold_ = declare_parameter<int>("weak_first_local_noise_threshold");
    roi_mode_ = declare_parameter<std::string>("roi_mode");
    visibility_error_threshold_ = declare_parameter<double>("visibility_error_threshold");
    visibility_warn_threshold_ = declare_parameter<double>("visibility_warn_threshold");
  }
  updater_.setHardwareID("dual_return_outlier_filter");
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &DualReturnOutlierFilterComponent::onVisibilityChecker);
  updater_.setPeriod(0.1);

  image_pub_ =
    image_transport::create_publisher(this, "dual_return_outlier_filter/debug/frequency_image");
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "dual_return_outlier_filter/debug/visibility", rclcpp::SensorDataQoS());
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "dual_return_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
  }
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DualReturnOutlierFilterComponent::paramCallback, this, _1));
}

void DualReturnOutlierFilterComponent::onVisibilityChecker(DiagnosticStatusWrapper & stat)
{
  // Add values
  stat.add("value", std::to_string(visibility_));

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (visibility_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (visibility_ < visibility_error_threshold_) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR: low visibility in dual outlier filter";
  } else if (visibility_ < visibility_warn_threshold_) {
    level = DiagnosticStatus::WARN;
    msg = "WARNING: low visibility in dual outlier filter";
  }
  stat.summary(level, msg);
}

void DualReturnOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  pcl::PointCloud<PointXYZIRCAEDT>::Ptr pcl_input(new pcl::PointCloud<PointXYZIRCAEDT>);
  pcl::fromROSMsg(*input, *pcl_input);

  uint32_t vertical_bins = vertical_bins_;
  uint32_t horizontal_bins = 36;
  float max_azimuth = 2 * M_PI;
  float min_azimuth = 0.0f;
  switch (roi_mode_map_[roi_mode_]) {
    case 2: {
      max_azimuth = max_azimuth_deg_ * (M_PI / 180.0);
      min_azimuth = min_azimuth_deg_ * (M_PI / 180.0);
      break;
    }

    default: {
      max_azimuth = 2 * M_PI;
      min_azimuth = 0.0f;
      break;
    }
  }

  uint32_t horizontal_resolution =
    static_cast<uint32_t>((max_azimuth - min_azimuth) / horizontal_bins);

  pcl::PointCloud<PointXYZIRCAEDT>::Ptr pcl_output(new pcl::PointCloud<PointXYZIRCAEDT>);
  pcl_output->points.reserve(pcl_input->points.size());

  std::vector<pcl::PointCloud<PointXYZIRCAEDT>> pcl_input_ring_array;
  std::vector<pcl::PointCloud<PointXYZIRCAEDT>> weak_first_pcl_input_ring_array;

  pcl::PointCloud<PointXYZIRCAEDT>::Ptr noise_output(new pcl::PointCloud<PointXYZIRCAEDT>);
  noise_output->points.reserve(pcl_input->points.size());
  pcl_input_ring_array.resize(
    vertical_bins);  // TODO(davidw): this is for Pandar 40 only, make dynamic
  weak_first_pcl_input_ring_array.resize(vertical_bins);

  // Split into 36 x 10 degree bins x 40 lines (TODO: change to dynamic)
  for (const auto & p : pcl_input->points) {
    if (p.return_type == ReturnType::DUAL_WEAK_FIRST) {
      weak_first_pcl_input_ring_array.at(p.channel).push_back(p);
    } else {
      pcl_input_ring_array.at(p.channel).push_back(p);
    }
  }

  float max_azimuth_diff = max_azimuth_diff_;
  cv::Mat frequency_image(cv::Size(horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  for (const auto & weak_first_single_ring : weak_first_pcl_input_ring_array) {
    if (weak_first_single_ring.points.size() < 2) {
      continue;
    }
    std::vector<float> deleted_azimuths;
    std::vector<float> deleted_distances;
    pcl::PointCloud<PointXYZIRCAEDT> temp_segment;

    bool keep_next = false;
    uint ring_id = weak_first_single_ring.points.front().channel;
    for (auto iter = std::begin(weak_first_single_ring) + 1;
         iter != std::end(weak_first_single_ring) - 1; ++iter) {
      const float min_dist = std::min(iter->distance, (iter + 1)->distance);
      const float max_dist = std::max(iter->distance, (iter + 1)->distance);
      float azimuth_diff = (iter + 1)->azimuth - iter->azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 2 * M_PI : azimuth_diff;

      if (max_dist < min_dist * weak_first_distance_ratio_ && azimuth_diff < max_azimuth_diff) {
        temp_segment.points.push_back(*iter);
        keep_next = true;
      } else if (keep_next) {
        temp_segment.points.push_back(*iter);
        keep_next = false;
        // Analyze segment points here
      } else {
        // Log the deleted azimuth and its distance for analysis
        switch (roi_mode_map_[roi_mode_]) {
          case 1:  // base_link xyz-ROI
          {
            if (
              iter->x > x_min_ && iter->x < x_max_ && iter->y > y_min_ && iter->y < y_max_ &&
              iter->z > z_min_ && iter->z < z_max_) {
              deleted_azimuths.push_back(iter->azimuth < 0.f ? 0.f : iter->azimuth);
              deleted_distances.push_back(iter->distance);
              noise_output->points.push_back(*iter);
            }
            break;
          }
          case 2: {
            if (
              iter->azimuth > min_azimuth && iter->azimuth < max_azimuth &&
              iter->distance < max_distance_) {
              deleted_azimuths.push_back(iter->azimuth < 0.f ? 0.f : iter->azimuth);
              noise_output->points.push_back(*iter);
              deleted_distances.push_back(iter->distance);
            }
            break;
          }
          default: {
            deleted_azimuths.push_back(iter->azimuth < 0.f ? 0.f : iter->azimuth);
            deleted_distances.push_back(iter->distance);
            noise_output->points.push_back(*iter);
            break;
          }
        }
      }
    }
    // Analyze last segment points here
    std::vector<int> noise_frequency(horizontal_bins, 0);
    uint current_deleted_index = 0;
    uint current_temp_segment_index = 0;
    for (uint i = 0; i < noise_frequency.size() - 1; i++) {
      if (deleted_azimuths.size() == 0) {
        continue;
      }
      while (current_deleted_index < deleted_azimuths.size() &&
             (uint)deleted_azimuths[current_deleted_index] <
               ((i + static_cast<uint>(min_azimuth / horizontal_resolution) + 1) *
                horizontal_resolution)) {
        noise_frequency[i] = noise_frequency[i] + 1;
        current_deleted_index++;
      }
      if (temp_segment.points.size() > 0) {
        while ((temp_segment.points[current_temp_segment_index].azimuth < 0.f
                  ? 0.f
                  : temp_segment.points[current_temp_segment_index].azimuth) <
                 ((i + 1 + static_cast<uint>(min_azimuth / horizontal_resolution)) *
                  horizontal_resolution) &&
               current_temp_segment_index < (temp_segment.points.size() - 1)) {
          if (noise_frequency[i] < weak_first_local_noise_threshold_) {
            pcl_output->points.push_back(temp_segment.points[current_temp_segment_index]);
          } else {
            switch (roi_mode_map_[roi_mode_]) {
              case 1: {
                if (
                  temp_segment.points[current_temp_segment_index].x < x_max_ &&
                  temp_segment.points[current_temp_segment_index].x > x_min_ &&
                  temp_segment.points[current_temp_segment_index].y > y_max_ &&
                  temp_segment.points[current_temp_segment_index].y < y_min_ &&
                  temp_segment.points[current_temp_segment_index].z < z_max_ &&
                  temp_segment.points[current_temp_segment_index].z > z_min_) {
                  noise_frequency[i] = noise_frequency[i] + 1;
                  noise_output->points.push_back(temp_segment.points[current_temp_segment_index]);
                }
                break;
              }
              case 2: {
                if (
                  temp_segment.points[current_temp_segment_index].azimuth < max_azimuth &&
                  temp_segment.points[current_temp_segment_index].azimuth > min_azimuth &&
                  temp_segment.points[current_temp_segment_index].distance < max_distance_) {
                  noise_frequency[i] = noise_frequency[i] + 1;
                  noise_output->points.push_back(temp_segment.points[current_temp_segment_index]);
                }
                break;
              }
              default: {
                noise_frequency[i] = noise_frequency[i] + 1;
                noise_output->points.push_back(temp_segment.points[current_temp_segment_index]);
                break;
              }
            }
          }
          current_temp_segment_index++;
          frequency_image.at<uchar>(ring_id, i) = noise_frequency[i];
        }
      }
    }
  }

  // Ring outlier filter for normal points
  for (const auto & input_ring : pcl_input_ring_array) {
    if (input_ring.size() < 2) {
      continue;
    }
    pcl::PointCloud<PointXYZIRCAEDT> temp_segment;
    bool keep_next = false;
    // uint ring_id = input_ring.points.front().ring;
    for (auto iter = std::begin(input_ring) + 1; iter != std::end(input_ring) - 1; ++iter) {
      const float min_dist = std::min(iter->distance, (iter + 1)->distance);
      const float max_dist = std::max(iter->distance, (iter + 1)->distance);
      float azimuth_diff = (iter + 1)->azimuth - iter->azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 2 * M_PI : azimuth_diff;

      if (max_dist < min_dist * general_distance_ratio_ && azimuth_diff < max_azimuth_diff) {
        temp_segment.points.push_back(*iter);
        keep_next = true;
      } else if (keep_next) {
        temp_segment.points.push_back(*iter);
        keep_next = false;
        // Analyze segment points here
      } else {
        // Log the deleted azimuth and its distance for analysis
        // deleted_azimuths.push_back(iter->azimuth < 0.f ? 0.f : iter->azimuth);
        // deleted_distances.push_back(iter->distance);
        noise_output->points.push_back(*iter);
      }
    }
    for (const auto & tmp_p : temp_segment.points) {
      pcl_output->points.push_back(tmp_p);
    }
  }

  // Threshold for diagnostics (tunable)
  cv::Mat binary_image;
  cv::inRange(frequency_image, weak_first_local_noise_threshold_, 255, binary_image);
  int num_pixels = cv::countNonZero(binary_image);
  float filled =
    static_cast<float>(num_pixels) / static_cast<float>(vertical_bins * horizontal_bins);
  visibility_ = 1.0f - filled;
  // Visualization of histogram
  cv::Mat frequency_image_colorized;
  // Multiply bins by four to get pretty colours
  cv::applyColorMap(frequency_image * 4, frequency_image_colorized, cv::COLORMAP_JET);
  sensor_msgs::msg::Image::SharedPtr frequency_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frequency_image_colorized).toImageMsg();
  frequency_image_msg->header = input->header;
  // Publish histogram image
  image_pub_.publish(frequency_image_msg);
  autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
  visibility_msg.data = (1.0f - filled);
  visibility_msg.stamp = now();
  visibility_pub_->publish(visibility_msg);

  // Publish noise points
  sensor_msgs::msg::PointCloud2 noise_output_msg;
  pcl::toROSMsg(*noise_output, noise_output_msg);
  noise_output_msg.header = input->header;
  noise_cloud_pub_->publish(noise_output_msg);

  // Publish filtered pointcloud
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult DualReturnOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "weak_first_distance_ratio", weak_first_distance_ratio_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new weak first distance ratio to: %f.", weak_first_distance_ratio_);
  }
  if (get_param(p, "general_distance_ratio", general_distance_ratio_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new general distance ratio to: %f.", general_distance_ratio_);
  }
  if (get_param(p, "weak_first_local_noise_threshold", weak_first_local_noise_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new weak first local noise threshold to: %d.",
      weak_first_local_noise_threshold_);
  }
  if (get_param(p, "vertical_bins", vertical_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new vertical_bins to: %d.", vertical_bins_);
  }
  if (get_param(p, "max_azimuth_diff", max_azimuth_diff_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max_azimuth_diff to: %f.", max_azimuth_diff_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::DualReturnOutlierFilterComponent)
