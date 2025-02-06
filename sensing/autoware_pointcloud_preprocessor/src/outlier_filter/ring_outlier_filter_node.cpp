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

#include "autoware/pointcloud_preprocessor/outlier_filter/ring_outlier_filter_node.hpp"

#include "autoware/point_types/types.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRADRT;

RingOutlierFilterComponent::RingOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RingOutlierFilter", options)
{
  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "ring_outlier_filter");
    {
      rclcpp::PublisherOptions pub_options;
      pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
      outlier_pointcloud_publisher_ =
        this->create_publisher<PointCloud2>("debug/ring_outlier_filter", 1, pub_options);
    }
    visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "ring_outlier_filter/debug/visibility", rclcpp::SensorDataQoS());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // set initial parameters
  {
    distance_ratio_ = declare_parameter<double>("distance_ratio");
    object_length_threshold_ = declare_parameter<double>("object_length_threshold");
    num_points_threshold_ = declare_parameter<int>("num_points_threshold");
    max_rings_num_ = static_cast<uint16_t>(declare_parameter<int64_t>("max_rings_num"));
    max_points_num_per_ring_ =
      static_cast<size_t>(declare_parameter<int64_t>("max_points_num_per_ring"));

    publish_outlier_pointcloud_ = declare_parameter<bool>("publish_outlier_pointcloud");

    min_azimuth_deg_ = declare_parameter<float>("min_azimuth_deg");
    max_azimuth_deg_ = declare_parameter<float>("max_azimuth_deg");
    max_distance_ = declare_parameter<float>("max_distance");
    vertical_bins_ = declare_parameter<int>("vertical_bins");
    horizontal_bins_ = declare_parameter<int>("horizontal_bins");
    noise_threshold_ = declare_parameter<int>("noise_threshold");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RingOutlierFilterComponent::paramCallback, this, _1));
}

// TODO(sykwer): Temporary Implementation: Rename this function to `filter()` when all the filter
// nodes conform to new API. Then delete the old `filter()` defined below.
void RingOutlierFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & unused_indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  if (unused_indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  stop_watch_ptr_->toc("processing_time", true);

  output.point_step = sizeof(OutputPointType);
  output.data.resize(output.point_step * input->width);
  size_t output_size = 0;

  pcl::PointCloud<InputPointType>::Ptr outlier_pcl(new pcl::PointCloud<InputPointType>);

  const auto input_channel_offset =
    input->fields.at(static_cast<size_t>(InputPointIndex::Channel)).offset;
  const auto input_azimuth_offset =
    input->fields.at(static_cast<size_t>(InputPointIndex::Azimuth)).offset;
  const auto input_distance_offset =
    input->fields.at(static_cast<size_t>(InputPointIndex::Distance)).offset;
  const auto input_intensity_offset =
    input->fields.at(static_cast<size_t>(InputPointIndex::Intensity)).offset;
  const auto input_return_type_offset =
    input->fields.at(static_cast<size_t>(InputPointIndex::ReturnType)).offset;

  std::vector<std::vector<size_t>> ring2indices;
  ring2indices.reserve(max_rings_num_);

  for (uint16_t i = 0; i < max_rings_num_; i++) {
    ring2indices.push_back(std::vector<size_t>());
    ring2indices.back().reserve(max_points_num_per_ring_);
  }

  for (size_t data_idx = 0; data_idx < input->data.size(); data_idx += input->point_step) {
    const uint16_t ring =
      *reinterpret_cast<const uint16_t *>(&input->data[data_idx + input_channel_offset]);
    ring2indices[ring].push_back(data_idx);
  }

  // walk range: [walk_first_idx, walk_last_idx]
  int walk_first_idx = 0;
  int walk_last_idx = -1;

  for (const auto & indices : ring2indices) {
    if (indices.size() < 2) continue;

    walk_first_idx = 0;
    walk_last_idx = -1;

    for (size_t idx = 0U; idx < indices.size() - 1; ++idx) {
      const size_t & current_data_idx = indices[idx];
      const size_t & next_data_idx = indices[idx + 1];
      walk_last_idx = idx;

      // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08)

      const float & current_azimuth =
        *reinterpret_cast<const float *>(&input->data[current_data_idx + input_azimuth_offset]);
      const float & next_azimuth =
        *reinterpret_cast<const float *>(&input->data[next_data_idx + input_azimuth_offset]);
      float azimuth_diff = next_azimuth - current_azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 2 * M_PI : azimuth_diff;

      const float & current_distance =
        *reinterpret_cast<const float *>(&input->data[current_data_idx + input_distance_offset]);
      const float & next_distance =
        *reinterpret_cast<const float *>(&input->data[next_data_idx + input_distance_offset]);

      if (
        std::max(current_distance, next_distance) <
          std::min(current_distance, next_distance) * distance_ratio_ &&
        azimuth_diff < 1.0 * (180.0 / M_PI)) {  // one degree
        continue;                               // Determined to be included in the same walk
      }

      if (isCluster(
            input, std::make_pair(indices[walk_first_idx], indices[walk_last_idx]),
            walk_last_idx - walk_first_idx + 1)) {
        for (int i = walk_first_idx; i <= walk_last_idx; i++) {
          auto output_ptr = reinterpret_cast<OutputPointType *>(&output.data[output_size]);
          auto input_ptr = reinterpret_cast<const InputPointType *>(&input->data[indices[i]]);

          if (transform_info.need_transform) {
            Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
            p = transform_info.eigen_transform * p;
            output_ptr->x = p[0];
            output_ptr->y = p[1];
            output_ptr->z = p[2];
          } else {
            output_ptr->x = input_ptr->x;
            output_ptr->y = input_ptr->y;
            output_ptr->z = input_ptr->z;
          }
          const std::uint8_t & intensity = *reinterpret_cast<const std::uint8_t *>(
            &input->data[indices[i] + input_intensity_offset]);
          output_ptr->intensity = intensity;

          const std::uint8_t & return_type = *reinterpret_cast<const std::uint8_t *>(
            &input->data[indices[i] + input_return_type_offset]);
          output_ptr->return_type = return_type;

          const std::uint8_t & channel = *reinterpret_cast<const std::uint8_t *>(
            &input->data[indices[i] + input_channel_offset]);
          output_ptr->channel = channel;

          output_size += output.point_step;
        }
      } else if (publish_outlier_pointcloud_) {
        for (int i = walk_first_idx; i <= walk_last_idx; i++) {
          auto input_ptr =
            reinterpret_cast<const InputPointType *>(&input->data[indices[walk_first_idx]]);
          InputPointType outlier_point = *input_ptr;

          if (transform_info.need_transform) {
            Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
            p = transform_info.eigen_transform * p;
            outlier_point.x = p[0];
            outlier_point.y = p[1];
            outlier_point.z = p[2];
          }

          outlier_pcl->push_back(outlier_point);
        }
      }

      walk_first_idx = idx + 1;
    }

    if (walk_first_idx > walk_last_idx) continue;

    if (isCluster(
          input, std::make_pair(indices[walk_first_idx], indices[walk_last_idx]),
          walk_last_idx - walk_first_idx + 1)) {
      for (int i = walk_first_idx; i <= walk_last_idx; i++) {
        auto output_ptr = reinterpret_cast<OutputPointType *>(&output.data[output_size]);
        auto input_ptr = reinterpret_cast<const InputPointType *>(&input->data[indices[i]]);

        if (transform_info.need_transform) {
          Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
          p = transform_info.eigen_transform * p;
          output_ptr->x = p[0];
          output_ptr->y = p[1];
          output_ptr->z = p[2];
        } else {
          output_ptr->x = input_ptr->x;
          output_ptr->y = input_ptr->y;
          output_ptr->z = input_ptr->z;
        }
        const float & intensity =
          *reinterpret_cast<const float *>(&input->data[indices[i] + input_intensity_offset]);
        output_ptr->intensity = intensity;

        const std::uint8_t & return_type = *reinterpret_cast<const std::uint8_t *>(
          &input->data[indices[i] + input_return_type_offset]);
        output_ptr->return_type = return_type;

        const std::uint8_t & channel =
          *reinterpret_cast<const std::uint8_t *>(&input->data[indices[i] + input_channel_offset]);
        output_ptr->channel = channel;

        output_size += output.point_step;
      }
    } else if (publish_outlier_pointcloud_) {
      for (int i = walk_first_idx; i < walk_last_idx; i++) {
        auto input_ptr = reinterpret_cast<const InputPointType *>(&input->data[indices[i]]);
        InputPointType outlier_point = *input_ptr;
        if (transform_info.need_transform) {
          Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
          p = transform_info.eigen_transform * p;
          outlier_point.x = p[0];
          outlier_point.y = p[1];
          outlier_point.z = p[2];
        }

        outlier_pcl->push_back(outlier_point);
      }
    }
  }

  setUpPointCloudFormat(input, output, output_size);

  if (publish_outlier_pointcloud_) {
    PointCloud2 outlier;
    pcl::toROSMsg(*outlier_pcl, outlier);
    outlier.header = input->header;
    outlier_pointcloud_publisher_->publish(outlier);

    autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
    visibility_msg.data = calculateVisibilityScore(outlier);
    visibility_msg.stamp = input->header.stamp;
    visibility_pub_->publish(visibility_msg);
  }

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - input->header.stamp).nanoseconds()))
        .count();

    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API
void RingOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

rcl_interfaces::msg::SetParametersResult RingOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "distance_ratio", distance_ratio_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance ratio to: %f.", distance_ratio_);
  }
  if (get_param(p, "object_length_threshold", object_length_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new object length threshold to: %f.", object_length_threshold_);
  }
  if (get_param(p, "num_points_threshold", num_points_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new num_points_threshold to: %d.", num_points_threshold_);
  }
  if (get_param(p, "publish_outlier_pointcloud", publish_outlier_pointcloud_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new publish_outlier_pointcloud to: %d.", publish_outlier_pointcloud_);
  }
  if (get_param(p, "vertical_bins", vertical_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new vertical_bins to: %d.", vertical_bins_);
  }
  if (get_param(p, "horizontal_bins", horizontal_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new horizontal_bins to: %d.", horizontal_bins_);
  }
  if (get_param(p, "noise_threshold", noise_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new noise_threshold to: %d.", noise_threshold_);
  }
  if (get_param(p, "max_azimuth_deg", max_azimuth_deg_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max_azimuth_deg to: %f.", max_azimuth_deg_);
  }
  if (get_param(p, "min_azimuth_deg", min_azimuth_deg_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new min_azimuth_deg to: %f.", min_azimuth_deg_);
  }
  if (get_param(p, "max_distance", max_distance_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max_distance to: %f.", max_distance_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void RingOutlierFilterComponent::setUpPointCloudFormat(
  const PointCloud2ConstPtr & input, PointCloud2 & formatted_points, size_t points_size)
{
  formatted_points.data.resize(points_size);
  // Note that `input->header.frame_id` is data before converted when `transform_info.need_transform
  // == true`
  formatted_points.header.frame_id =
    !tf_input_frame_.empty() ? tf_input_frame_ : tf_input_orig_frame_;
  formatted_points.height = 1;
  formatted_points.width =
    static_cast<uint32_t>(formatted_points.data.size() / formatted_points.point_step);
  formatted_points.is_bigendian = input->is_bigendian;
  formatted_points.is_dense = input->is_dense;

  // This is a hack to get the correct fields in the output point cloud without creating the fields
  // manually
  sensor_msgs::msg::PointCloud2 msg_aux;
  pcl::toROSMsg(pcl::PointCloud<OutputPointType>(), msg_aux);
  formatted_points.fields = msg_aux.fields;
}

float RingOutlierFilterComponent::calculateVisibilityScore(
  const sensor_msgs::msg::PointCloud2 & input)
{
  pcl::PointCloud<InputPointType>::Ptr input_cloud(new pcl::PointCloud<InputPointType>);
  pcl::fromROSMsg(input, *input_cloud);

  const uint32_t vertical_bins = vertical_bins_;
  const uint32_t horizontal_bins = horizontal_bins_;
  const float max_azimuth = max_azimuth_deg_ * (M_PI / 180.f);
  const float min_azimuth = min_azimuth_deg_ * (M_PI / 180.f);

  const uint32_t horizontal_resolution =
    static_cast<uint32_t>((max_azimuth - min_azimuth) / horizontal_bins);

  std::vector<pcl::PointCloud<InputPointType>> ring_point_clouds(vertical_bins);
  cv::Mat frequency_image(cv::Size(horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  // Split points into rings
  for (const auto & point : input_cloud->points) {
    ring_point_clouds.at(point.channel).push_back(point);
  }

  // Calculate frequency for each bin in each ring
  for (const auto & ring_points : ring_point_clouds) {
    if (ring_points.empty()) continue;

    const uint ring_id = ring_points.front().channel;
    std::vector<int> frequency_in_ring(horizontal_bins, 0);

    for (const auto & point : ring_points.points) {
      if (point.azimuth < min_azimuth || point.azimuth >= max_azimuth) continue;
      if (point.distance >= max_distance_) continue;

      const uint bin_index =
        static_cast<uint>((point.azimuth - min_azimuth) / horizontal_resolution);

      frequency_in_ring[bin_index]++;
      frequency_in_ring[bin_index] =
        std::min(frequency_in_ring[bin_index], 255);  // Ensure value is within uchar range

      frequency_image.at<uchar>(ring_id, bin_index) =
        static_cast<uchar>(frequency_in_ring[bin_index]);
    }
  }

  cv::Mat binary_image;
  cv::inRange(frequency_image, noise_threshold_, 255, binary_image);

  const int num_pixels = cv::countNonZero(frequency_image);
  const float num_filled_pixels =
    static_cast<float>(num_pixels) / static_cast<float>(vertical_bins * horizontal_bins);

  return 1.0f - num_filled_pixels;
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::RingOutlierFilterComponent)
