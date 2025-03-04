// Copyright 2025 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/pointcloud_densifier/pointcloud_densifier_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>  
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  

#include <memory>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

PointCloudDensifierNode::PointCloudDensifierNode(const rclcpp::NodeOptions & options)
: Filter("PointCloudDensifier", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  // Initialize debug tools
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Set initial parameters
  {
    param_.num_previous_frames = declare_parameter<int>("num_previous_frames", 1);
    param_.x_min = declare_parameter<double>("x_min", 80.0);
    param_.x_max = declare_parameter<double>("x_max", 200.0);
    param_.y_min = declare_parameter<double>("y_min", -20.0);
    param_.y_max = declare_parameter<double>("y_max", 20.0);
    param_.grid_resolution = declare_parameter<double>("grid_resolution", 0.3);

    if (param_.num_previous_frames < 0) {
      RCLCPP_WARN(get_logger(), "num_previous_frames must be non-negative. Setting to 0.");
      param_.num_previous_frames = 0;
    }
    if (param_.x_min >= param_.x_max || param_.y_min >= param_.y_max) {
      RCLCPP_ERROR(get_logger(), "Invalid ROI bounds: x_min must be less than x_max, and y_min must be less than y_max");
      throw std::invalid_argument("Invalid ROI bounds");
    }
    if (param_.grid_resolution <= 0.0) {
      RCLCPP_ERROR(get_logger(), "grid_resolution must be positive");
      throw std::invalid_argument("Invalid grid resolution");
    }
  }

  // Set parameter service callback
  {
    using std::placeholders::_1;
    set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&PointCloudDensifierNode::paramCallback, this, _1));
  }
}

// Legacy filter implementation - for backward compatibility
void PointCloudDensifierNode::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  // This is not used as we're using faster_filter directly
  (void)input;
  (void)indices;
  (void)output;
}

// Optimized filter implementation using TransformInfo
void PointCloudDensifierNode::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  // No need this transformer 
  (void)transform_info;

  auto far_front_pointcloud_ptr = filterPointCloudByROI(input, indices);
  
  // Build occupancy grid from the current filtered cloud
  OccupancyGrid occupancy_grid(
    param_.x_min, param_.x_max, param_.y_min, param_.y_max, param_.grid_resolution);
  occupancy_grid.updateOccupancy(*far_front_pointcloud_ptr);


  output = *input;  

  transformAndMergePreviousClouds(input, occupancy_grid, output);

  storeCurrentCloud(far_front_pointcloud_ptr);

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

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudDensifierNode::filterPointCloudByROI(
  const PointCloud2ConstPtr & input_cloud, const IndicesPtr & indices)
{
  auto filtered_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  
  filtered_cloud->header = input_cloud->header;
  filtered_cloud->height = 1;  
  filtered_cloud->fields = input_cloud->fields;
  filtered_cloud->is_bigendian = input_cloud->is_bigendian;
  filtered_cloud->point_step = input_cloud->point_step;
  filtered_cloud->is_dense = input_cloud->is_dense;
  
  int x_offset = input_cloud->fields[pcl::getFieldIndex(*input_cloud, "x")].offset;
  int y_offset = input_cloud->fields[pcl::getFieldIndex(*input_cloud, "y")].offset;
  
  // Pre-allocate data for the filtered cloud
  filtered_cloud->data.resize(input_cloud->data.size());
  size_t output_size = 0;
  
  if (indices && !indices->empty()) {
    for (const auto & idx : *indices) {
      const size_t data_offset = idx * input_cloud->point_step;
      
      float x, y;
      std::memcpy(&x, &input_cloud->data[data_offset + x_offset], sizeof(float));
      std::memcpy(&y, &input_cloud->data[data_offset + y_offset], sizeof(float));
      
      if (x > param_.x_min && x < param_.x_max && y > param_.y_min && y < param_.y_max) {
        // Copy point data if within ROI
        std::memcpy(
          &filtered_cloud->data[output_size],
          &input_cloud->data[data_offset],
          input_cloud->point_step
        );
        output_size += input_cloud->point_step;
      }
    }
  } else {
    for (size_t i = 0; i < input_cloud->width * input_cloud->height; ++i) {
      const size_t data_offset = i * input_cloud->point_step;
      
      // Get x,y coordinates
      float x, y;
      std::memcpy(&x, &input_cloud->data[data_offset + x_offset], sizeof(float));
      std::memcpy(&y, &input_cloud->data[data_offset + y_offset], sizeof(float));
      
      if (x > param_.x_min && x < param_.x_max && y > param_.y_min && y < param_.y_max) {
        // Copy point data if within ROI
        std::memcpy(
          &filtered_cloud->data[output_size],
          &input_cloud->data[data_offset],
          input_cloud->point_step
        );
        output_size += input_cloud->point_step;
      }
    }
  }
  
  // Set width based on actual number of points
  filtered_cloud->width = output_size / filtered_cloud->point_step;
  filtered_cloud->row_step = filtered_cloud->width * filtered_cloud->point_step;
  filtered_cloud->data.resize(output_size);
  
  return filtered_cloud;
}

void PointCloudDensifierNode::transformAndMergePreviousClouds(
  const PointCloud2ConstPtr & current_msg,
  const OccupancyGrid & occupancy_grid,
  PointCloud2 & combined_cloud)
{
  int x_offset = current_msg->fields[pcl::getFieldIndex(*current_msg, "x")].offset;
  int y_offset = current_msg->fields[pcl::getFieldIndex(*current_msg, "y")].offset;
  
  for (const auto& previous_cloud : previous_pointclouds_) {
    if (!previous_cloud || previous_cloud->data.empty()) {
      continue;
    }
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      tf2::TimePoint current_time_point = tf2::TimePoint(
        std::chrono::nanoseconds(current_msg->header.stamp.nanosec) +
        std::chrono::seconds(current_msg->header.stamp.sec));
      tf2::TimePoint prev_time_point = tf2::TimePoint(
        std::chrono::nanoseconds(previous_cloud->header.stamp.nanosec) +
        std::chrono::seconds(previous_cloud->header.stamp.sec));
      transform_stamped = tf_buffer_->lookupTransform(
        current_msg->header.frame_id,
        current_time_point,
        previous_cloud->header.frame_id,
        prev_time_point,
        "map"  
      );
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Could not transform point cloud: %s", ex.what());
      continue;
    }
    
    // Validate and obtain the transformation
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped);
    if (!isValidTransform(transform_eigen.matrix())) {
      RCLCPP_WARN(get_logger(), "Invalid transform matrix, skipping point cloud");
      continue;
    }
    
    // Transform previous point cloud to current frame
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    tf2::doTransform(*previous_cloud, transformed_cloud, transform_stamped);
    
    // Get transformed cloud field offsets
    x_offset = transformed_cloud.fields[pcl::getFieldIndex(transformed_cloud, "x")].offset;
    y_offset = transformed_cloud.fields[pcl::getFieldIndex(transformed_cloud, "y")].offset;
    
    size_t original_size = combined_cloud.data.size();
    combined_cloud.data.resize(original_size + transformed_cloud.data.size());
    size_t output_size = original_size;
    
    // Add previous points only if they fall into occupied grid cells
    for (size_t i = 0; i < transformed_cloud.width; ++i) {
      size_t data_offset = i * transformed_cloud.point_step;
      
      float x, y;
      std::memcpy(&x, &transformed_cloud.data[data_offset + x_offset], sizeof(float));
      std::memcpy(&y, &transformed_cloud.data[data_offset + y_offset], sizeof(float));
      
      if (occupancy_grid.isOccupied(x, y)) {
        std::memcpy(
          &combined_cloud.data[output_size],
          &transformed_cloud.data[data_offset],
          transformed_cloud.point_step
        );
        output_size += transformed_cloud.point_step;
      }
    }
    
    combined_cloud.data.resize(output_size);
  }
  
  combined_cloud.width = combined_cloud.data.size() / combined_cloud.point_step;
  combined_cloud.row_step = combined_cloud.width * combined_cloud.point_step;
  combined_cloud.height = 1;  
}

void PointCloudDensifierNode::storeCurrentCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & filtered_cloud)
{
  if (previous_pointclouds_.size() >= static_cast<size_t>(param_.num_previous_frames)) {
    previous_pointclouds_.pop_front();
  }
  previous_pointclouds_.push_back(filtered_cloud);
}

bool PointCloudDensifierNode::isValidTransform(const Eigen::Matrix4d & transform) const
{
  return transform.allFinite() && 
         std::abs(transform.determinant() - 1.0) < 1e-3; // Check if it's a proper rigid transformation
}

rcl_interfaces::msg::SetParametersResult PointCloudDensifierNode::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  DensifierParam new_param = param_;

  if (get_param(p, "num_previous_frames", new_param.num_previous_frames)) {
    if (new_param.num_previous_frames < 0) {
      new_param.num_previous_frames = 0;
      RCLCPP_WARN(get_logger(), "num_previous_frames must be non-negative. Setting to 0.");
    }
  }

  bool update_grid = false;
  if (get_param(p, "x_min", new_param.x_min)) update_grid = true;
  if (get_param(p, "x_max", new_param.x_max)) update_grid = true;
  if (get_param(p, "y_min", new_param.y_min)) update_grid = true;
  if (get_param(p, "y_max", new_param.y_max)) update_grid = true;
  if (get_param(p, "grid_resolution", new_param.grid_resolution)) update_grid = true;

  if (update_grid) {
    // Validate grid parameters
    if (new_param.x_min >= new_param.x_max || new_param.y_min >= new_param.y_max) {
      RCLCPP_ERROR(get_logger(), "Invalid ROI bounds: x_min must be less than x_max, and y_min must be less than y_max");
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Invalid ROI bounds";
      return result;
    }
    if (new_param.grid_resolution <= 0.0) {
      RCLCPP_ERROR(get_logger(), "grid_resolution must be positive");
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Invalid grid resolution";
      return result;
    }
  }

  param_ = new_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PointCloudDensifierNode)
