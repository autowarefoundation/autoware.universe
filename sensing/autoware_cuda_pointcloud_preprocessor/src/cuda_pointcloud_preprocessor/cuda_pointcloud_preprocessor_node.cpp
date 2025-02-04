// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <autoware/point_types/types.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cuda_runtime.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <chrono>  // TODO(knzo25): remove this
#include <limits>  // TODO(knzo25): remove this
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
using sensor_msgs::msg::PointCloud2;

CudaPointcloudPreprocessorNode::CudaPointcloudPreprocessorNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_pointcloud_preprocessor", node_options),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_)
{
  using std::placeholders::_1;

  // Parameters
  base_frame_ = declare_parameter<std::string>("base_frame");

  RingOutlierFilterParameters ring_outlier_filter_parameters;
  ring_outlier_filter_parameters.distance_ratio = declare_parameter<float>("distance_ratio");
  ring_outlier_filter_parameters.object_length_threshold =
    declare_parameter<float>("object_length_threshold");
  ring_outlier_filter_parameters.num_points_threshold =
    declare_parameter<int>("num_points_threshold");

  const auto crop_box_min_x_vector = declare_parameter<std::vector<double>>("crop_box.min_x");
  const auto crop_box_min_y_vector = declare_parameter<std::vector<double>>("crop_box.min_y");
  const auto crop_box_min_z_vector = declare_parameter<std::vector<double>>("crop_box.min_z");

  const auto crop_box_max_x_vector = declare_parameter<std::vector<double>>("crop_box.max_x");
  const auto crop_box_max_y_vector = declare_parameter<std::vector<double>>("crop_box.max_y");
  const auto crop_box_max_z_vector = declare_parameter<std::vector<double>>("crop_box.max_z");

  if (
    crop_box_min_x_vector.size() != crop_box_min_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_min_z_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_x_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_y_vector.size() ||
    crop_box_min_x_vector.size() != crop_box_max_z_vector.size()) {
    throw std::runtime_error("Crop box parameters must have the same size");
  }

  std::vector<CropBoxParameters> crop_box_parameters;

  for (std::size_t i = 0; i < crop_box_min_x_vector.size(); i++) {
    CropBoxParameters parameters;
    parameters.min_x = crop_box_min_x_vector[i];
    parameters.min_y = crop_box_min_y_vector[i];
    parameters.min_z = crop_box_min_z_vector[i];
    parameters.max_x = crop_box_max_x_vector[i];
    parameters.max_y = crop_box_max_y_vector[i];
    parameters.max_z = crop_box_max_z_vector[i];
    crop_box_parameters.push_back(parameters);
  }

  bool use_3d_undistortion = declare_parameter<bool>("use_3d_distortion_correction");
  bool use_imu = declare_parameter<bool>("use_imu");

  // Publisher
  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  // Subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&CudaPointcloudPreprocessorNode::pointcloudCallback, this, std::placeholders::_1),
    sub_options);
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(&CudaPointcloudPreprocessorNode::twistCallback, this, std::placeholders::_1),
    sub_options);

  if (use_imu) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "~/input/imu", 10,
      std::bind(&CudaPointcloudPreprocessorNode::imuCallback, this, std::placeholders::_1),
      sub_options);
  }

  cuda_pointcloud_preprocessor_ = std::make_unique<CudaPointcloudPreprocessor>();
  cuda_pointcloud_preprocessor_->setRingOutlierFilterParameters(ring_outlier_filter_parameters);
  cuda_pointcloud_preprocessor_->setCropBoxParameters(crop_box_parameters);
  cuda_pointcloud_preprocessor_->set3DUndistortion(use_3d_undistortion);

  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;

    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "cuda_pointcloud_preprocessor");
    stop_watch_ptr_->tic("processing_time");
  }

  // New organized pointcloud method
  test_num_rings_ = 1;
  test_max_points_per_ring_ = 1;
  ring_index_device_ = cuda_blackboard::make_unique<std::int32_t[]>(test_num_rings_);
  indexes_tensor_device_ =
    cuda_blackboard::make_unique<std::uint32_t[]>(test_num_rings_ * test_max_points_per_ring_);
  sorted_indexes_tensor_device_ =
    cuda_blackboard::make_unique<std::uint32_t[]>(test_num_rings_ * test_max_points_per_ring_);
  segment_offsets_device_ = cuda_blackboard::make_unique<std::int32_t[]>(test_num_rings_ + 1);
  max_rings_device_ = cuda_blackboard::make_unique<std::int32_t>();
  max_points_per_ring_device_ = cuda_blackboard::make_unique<std::int32_t>();
  organized_points_device_ =
    cuda_blackboard::make_unique<InputPointType[]>(test_num_rings_ * test_max_points_per_ring_);

  cudaStreamCreate(&stream_);
  cudaMemsetAsync(max_rings_device_.get(), 0, sizeof(std::int32_t), stream_);
  cudaMemsetAsync(max_points_per_ring_device_.get(), 0, sizeof(std::int32_t), stream_);

  sort_workspace_bytes_ = querySortWorkspace(
    test_num_rings_ * test_max_points_per_ring_, test_num_rings_, segment_offsets_device_.get(),
    indexes_tensor_device_.get(), sorted_indexes_tensor_device_.get());
  sort_workspace_device_ = cuda_blackboard::make_unique<std::uint8_t[]>(sort_workspace_bytes_);

  capacity_ = 8192;
  points_device_ = cuda_blackboard::make_unique<InputPointType[]>(capacity_);
}

bool CudaPointcloudPreprocessorNode::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return false;
  }
  return true;
}

void CudaPointcloudPreprocessorNode::twistCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  twist_queue_.push_back(*twist_msg_ptr);

  while (!twist_queue_.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg_ptr->header.stamp)) {
      twist_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_queue_.front().header.stamp) <
      rclcpp::Time(twist_msg_ptr->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue_.pop_front();
    }
    break;
  }
}

void CudaPointcloudPreprocessorNode::imuCallback(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  tf2::Transform tf2_imu_link_to_base_link{};
  getTransform(base_frame_, imu_msg->header.frame_id, &tf2_imu_link_to_base_link);
  geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf_base2imu_ptr->transform.rotation = tf2::toMsg(tf2_imu_link_to_base_link.getRotation());

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_base2imu_ptr);
  transformed_angular_velocity.header = imu_msg->header;
  angular_velocity_queue_.push_back(transformed_angular_velocity);

  while (!angular_velocity_queue_.empty()) {
    // for rosbag replay
    if (
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) >
      rclcpp::Time(imu_msg->header.stamp)) {
      angular_velocity_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) <
      rclcpp::Time(imu_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      angular_velocity_queue_.pop_front();
    }
    break;
  }
}

void CudaPointcloudPreprocessorNode::estimatePointcloudRingInfo(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr)
{
  const autoware::point_types::PointXYZIRCAEDT * input_buffer =
    reinterpret_cast<const autoware::point_types::PointXYZIRCAEDT *>(
      input_pointcloud_msg_ptr->data.data());

  std::size_t max_ring = 0;

  for (std::size_t i = 0; i < input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height;
       i++) {
    const autoware::point_types::PointXYZIRCAEDT & point = input_buffer[i];
    const std::size_t ring = static_cast<std::size_t>(point.channel);
    max_ring = std::max(max_ring, ring);
  }

  // Set max rings to the next power of two
  num_rings_ = std::pow(2, std::ceil(std::log2(max_ring + 1)));
  num_rings_ = std::max(num_rings_, static_cast<std::size_t>(16));
  std::vector<std::size_t> ring_points(num_rings_, 0);

  for (std::size_t i = 0; i < input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height;
       i++) {
    const autoware::point_types::PointXYZIRCAEDT & point = input_buffer[i];
    const std::size_t ring = point.channel;
    ring_points[ring]++;
  }

  // Set max points per ring to the next multiple of 512
  max_points_per_ring_ = *std::max_element(ring_points.begin(), ring_points.end());
  max_points_per_ring_ = std::max(max_points_per_ring_, static_cast<std::size_t>(512));
  max_points_per_ring_ = (max_points_per_ring_ + 511) / 512 * 512;

  next_ring_index_.resize(num_rings_);
  std::fill(next_ring_index_.begin(), next_ring_index_.end(), 0);
  host_buffer_ = cuda_blackboard::make_host_unique<autoware::point_types::PointXYZIRCAEDT[]>(
    num_rings_ * max_points_per_ring_);

  input_pointcloud_ptr_ = std::make_shared<cuda_blackboard::CudaPointCloud2>();
  input_pointcloud_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
    num_rings_ * max_points_per_ring_ * sizeof(autoware::point_types::PointXYZIRCAEDT));
  input_pointcloud_ptr_->width = max_points_per_ring_;
  input_pointcloud_ptr_->height = num_rings_;
  input_pointcloud_ptr_->point_step = sizeof(autoware::point_types::PointXYZIRCAEDT);
  input_pointcloud_ptr_->row_step =
    max_points_per_ring_ * sizeof(autoware::point_types::PointXYZIRCAEDT);
  input_pointcloud_ptr_->is_dense = input_pointcloud_msg_ptr->is_dense;
  input_pointcloud_ptr_->fields = input_pointcloud_msg_ptr->fields;
  input_pointcloud_ptr_->header = input_pointcloud_msg_ptr->header;

  RCLCPP_INFO_STREAM(
    get_logger(), "Estimated rings: " << num_rings_
                                      << ", max_points_per_ring: " << max_points_per_ring_
                                      << ". This should only be done during the first iterations. "
                                         "Otherwise, performance will be affected.");
}

bool CudaPointcloudPreprocessorNode::orderPointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr)
{
  const autoware::point_types::PointXYZIRCAEDT * input_buffer =
    reinterpret_cast<const autoware::point_types::PointXYZIRCAEDT *>(
      input_pointcloud_msg_ptr->data.data());

  bool ring_overflow = false;
  bool point_overflow = false;

  autoware::point_types::PointXYZIRCAEDT * buffer = host_buffer_.get();

  for (std::size_t i = 0; i < input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height;
       i++) {
    const autoware::point_types::PointXYZIRCAEDT & point = input_buffer[i];
    const std::size_t raw_ring = point.channel;
    const std::size_t ring = raw_ring % num_rings_;

    const std::size_t raw_index = next_ring_index_[ring];
    const std::size_t index = raw_index % max_points_per_ring_;

    ring_overflow |= raw_ring >= num_rings_;
    point_overflow |= raw_index >= max_points_per_ring_;

    buffer[ring * max_points_per_ring_ + index] = point;
    next_ring_index_[ring] = raw_index + 1;
  }

  return !ring_overflow && !point_overflow;
}

void CudaPointcloudPreprocessorNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr)
{
  // NEW Cuda version
  if (input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height > capacity_) {
    capacity_ = input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height;
    capacity_ = (capacity_ + 1024) / 1024 * 1024;
    points_device_ = cuda_blackboard::make_unique<InputPointType[]>(capacity_);
  }

  double cuda_t0 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch())
                            .count();

  cudaMemcpyAsync(
    points_device_.get(), input_pointcloud_msg_ptr->data.data(),
    input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height * sizeof(InputPointType),
    cudaMemcpyHostToDevice, stream_);
  cudaStreamSynchronize(stream_);

  double cuda_t1 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch())
                            .count();

  // Estimate the number of rings and max points per ring
  cudaMemsetAsync(ring_index_device_.get(), 0, test_num_rings_ * sizeof(std::int32_t), stream_);
  cudaMemsetAsync(
    indexes_tensor_device_.get(), 0xFF,
    test_num_rings_ * test_max_points_per_ring_ * sizeof(std::uint32_t), stream_);
  organizeLaunch(
    points_device_.get(), indexes_tensor_device_.get(), ring_index_device_.get(), test_num_rings_,
    max_rings_device_.get(), test_max_points_per_ring_, max_points_per_ring_device_.get(),
    input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height, stream_);
  cudaStreamSynchronize(stream_);

  std::int32_t max_ring_value;
  std::int32_t max_points_per_ring;
  cudaMemcpyAsync(
    &max_ring_value, max_rings_device_.get(), sizeof(std::int32_t), cudaMemcpyDeviceToHost,
    stream_);
  cudaMemcpyAsync(
    &max_points_per_ring, max_points_per_ring_device_.get(), sizeof(std::int32_t),
    cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  if (max_ring_value >= test_num_rings_ || max_points_per_ring > test_max_points_per_ring_) {
    RCLCPP_WARN(
      get_logger(), "Current ring num: %d, Current Max points per ring: %d", test_num_rings_,
      test_max_points_per_ring_);
    test_num_rings_ = max_ring_value + 1;
    test_max_points_per_ring_ =
      std::max((max_points_per_ring + 511) / 512 * 512, 512);  // Re think this equation
    ring_index_device_ = cuda_blackboard::make_unique<std::int32_t[]>(test_num_rings_);
    indexes_tensor_device_ =
      cuda_blackboard::make_unique<std::uint32_t[]>(test_num_rings_ * test_max_points_per_ring_);
    sorted_indexes_tensor_device_ =
      cuda_blackboard::make_unique<std::uint32_t[]>(test_num_rings_ * test_max_points_per_ring_);
    segment_offsets_device_ = cuda_blackboard::make_unique<std::int32_t[]>(test_num_rings_ + 1);
    organized_points_device_ =
      cuda_blackboard::make_unique<InputPointType[]>(test_num_rings_ * test_max_points_per_ring_);
    RCLCPP_WARN(
      get_logger(), "NEW Max ring value: %d, Max points per ring: %d", max_ring_value,
      test_max_points_per_ring_);

    std::vector<std::int32_t> segment_offsets_host(test_num_rings_ + 1);
    for (std::size_t i = 0; i < test_num_rings_ + 1; i++) {
      segment_offsets_host[i] = i * test_max_points_per_ring_;
    }
    cudaMemcpyAsync(
      segment_offsets_device_.get(), segment_offsets_host.data(),
      (test_num_rings_ + 1) * sizeof(std::int32_t), cudaMemcpyHostToDevice, stream_);

    cudaMemsetAsync(ring_index_device_.get(), 0, test_num_rings_ * sizeof(std::int32_t), stream_);
    cudaMemsetAsync(
      indexes_tensor_device_.get(), 0xFF,
      test_num_rings_ * test_max_points_per_ring_ * sizeof(std::int32_t), stream_);

    sort_workspace_bytes_ = querySortWorkspace(
      test_num_rings_ * test_max_points_per_ring_, test_num_rings_, segment_offsets_device_.get(),
      indexes_tensor_device_.get(), sorted_indexes_tensor_device_.get());
    sort_workspace_device_ = cuda_blackboard::make_unique<std::uint8_t[]>(sort_workspace_bytes_);

    organizeLaunch(
      points_device_.get(), indexes_tensor_device_.get(), ring_index_device_.get(), test_num_rings_,
      max_rings_device_.get(), test_max_points_per_ring_, max_points_per_ring_device_.get(),
      input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height, stream_);
    // cudaStreamSynchronize(stream_);
  }

  /* std::vector<std::int32_t> ring_index_host(test_num_rings_);
  cudaMemcpyAsync(
    ring_index_host.data(), ring_index_device_.get(), test_num_rings_ * sizeof(std::int32_t),
  cudaMemcpyDeviceToHost, stream_); cudaStreamSynchronize(stream_);

  for (std::size_t i = 0; i < test_num_rings_; i++) {
    if (ring_index_host[i] > 0) {
      std::cout << "Ring id=" << i << " has " << ring_index_host[i] << " points" << std::endl;
    }
  } */

  sortLaunch(
    test_num_rings_ * test_max_points_per_ring_, test_num_rings_, segment_offsets_device_.get(),
    indexes_tensor_device_.get(), sorted_indexes_tensor_device_.get(), sort_workspace_device_.get(),
    sort_workspace_bytes_, stream_);
  cudaStreamSynchronize(stream_);

  /* std::vector<std::uint32_t> indexes_tensor_host(test_num_rings_ * test_max_points_per_ring_);
  cudaMemcpyAsync(
    indexes_tensor_host.data(), sorted_indexes_tensor_device_.get(),
    test_num_rings_ * test_max_points_per_ring_ * sizeof(std::uint32_t), cudaMemcpyDeviceToHost,
  stream_); cudaStreamSynchronize(stream_);

  if (test_num_rings_ < 64) {
    for (std::size_t i = 0; i < test_num_rings_; i++) {
      std::cout << "Ring id=" << i << "\n\t";
      for (std::size_t j = 0; j < test_max_points_per_ring_; j++) {
        if (indexes_tensor_host[i * test_max_points_per_ring_ + j] <
  std::numeric_limits<std::uint32_t>::max()) { std::cout << indexes_tensor_host[i *
  test_max_points_per_ring_ + j] << " ";
        }
      }
      std::cout << std::endl << std::flush;
    }
  } */

  gatherLaunch(
    points_device_.get(), sorted_indexes_tensor_device_.get(), organized_points_device_.get(),
    test_num_rings_, test_max_points_per_ring_, stream_);
  cudaStreamSynchronize(
    stream_);  // this will not be needed in the final implementation. for now we benchmark

  double cuda_t2 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch())
                            .count();

  if (iteration_ >= 0) {
    organize_time_cuda_ms_ += (cuda_t1 - cuda_t0);
    copy_time_cuda_ms_ += (cuda_t2 - cuda_t1);
  }

  stop_watch_ptr_->toc("processing_time", true);

  // TODO(knzo25): check the pointcloud layout at least once

  assert(input_pointcloud_msg_ptr->point_step == sizeof(autoware::point_types::PointXYZIRCAEDT));

  if (num_rings_ == 0 || max_points_per_ring_ == 0) {
    estimatePointcloudRingInfo(input_pointcloud_msg_ptr);
  }

  double t1 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::high_resolution_clock::now().time_since_epoch())
                       .count();
  if (!orderPointcloud(input_pointcloud_msg_ptr)) {
    estimatePointcloudRingInfo(input_pointcloud_msg_ptr);
    orderPointcloud(input_pointcloud_msg_ptr);
  }
  double t2 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::high_resolution_clock::now().time_since_epoch())
                       .count();

  // Copy to cuda memory
  cudaMemcpy(
    input_pointcloud_ptr_->data.get(), host_buffer_.get(),
    num_rings_ * max_points_per_ring_ * sizeof(autoware::point_types::PointXYZIRCAEDT),
    cudaMemcpyHostToDevice);

  double t3 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::high_resolution_clock::now().time_since_epoch())
                       .count();

  if (iteration_ >= 0) {
    organize_time_ms_ += (t2 - t1);
    copy_time_ms_ += (t3 - t2);
  }

  // Test -> this would replace the cpu version
  cudaMemcpyAsync(
    input_pointcloud_ptr_->data.get(), organized_points_device_.get(),
    test_num_rings_ * test_max_points_per_ring_ * sizeof(autoware::point_types::PointXYZIRCAEDT),
    cudaMemcpyHostToDevice, stream_);
  input_pointcloud_ptr_->height = test_num_rings_;
  input_pointcloud_ptr_->width = test_max_points_per_ring_;
  cudaStreamSynchronize(stream_);

  // TEST CODE
  {
    std::vector<autoware::point_types::PointXYZIRCAEDT> host_buffer_test(
      test_num_rings_ * test_max_points_per_ring_);
    cudaMemcpyAsync(
      host_buffer_test.data(), organized_points_device_.get(),
      test_num_rings_ * test_max_points_per_ring_ * sizeof(autoware::point_types::PointXYZIRCAEDT),
      cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    if (
      test_num_rings_ > 1 && test_max_points_per_ring_ == max_points_per_ring_ &&
      test_num_rings_ == num_rings_) {
      for (std::size_t i = 0; i < num_rings_; i++) {
        for (std::size_t j = 0; j < max_points_per_ring_; j++) {
          // Compare the implementations
          if (
            (host_buffer_.get()[i * max_points_per_ring_ + j].x !=
               host_buffer_test[i * max_points_per_ring_ + j].x ||
             host_buffer_.get()[i * max_points_per_ring_ + j].y !=
               host_buffer_test[i * max_points_per_ring_ + j].y ||
             host_buffer_.get()[i * max_points_per_ring_ + j].z !=
               host_buffer_test[i * max_points_per_ring_ + j].z) &&
            host_buffer_test[i * max_points_per_ring_ + j].distance > 0) {
            RCLCPP_ERROR(get_logger(), "Mismatch at ring=%d, point=%d", i, j);
            RCLCPP_ERROR(
              get_logger(), "\thost_buffer_test: x=%.2f y=%.2f z=%.2f channel=%d distance=%.2f",
              host_buffer_test[i * max_points_per_ring_ + j].x,
              host_buffer_test[i * max_points_per_ring_ + j].y,
              host_buffer_test[i * max_points_per_ring_ + j].z,
              host_buffer_test[i * max_points_per_ring_ + j].channel,
              host_buffer_test[i * max_points_per_ring_ + j].distance);
            RCLCPP_ERROR(
              get_logger(), "\thost_buffer: x=%.2f y=%.2f z=%.2f channel=%d distance=%.2f",
              host_buffer_.get()[i * max_points_per_ring_ + j].x,
              host_buffer_.get()[i * max_points_per_ring_ + j].y,
              host_buffer_.get()[i * max_points_per_ring_ + j].z,
              host_buffer_.get()[i * max_points_per_ring_ + j].channel,
              host_buffer_.get()[i * max_points_per_ring_ + j].distance);
          }
        }
      }
    }
  }

  input_pointcloud_ptr_->header = input_pointcloud_msg_ptr->header;

  cudaPointcloudCallback(input_pointcloud_ptr_);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    double now_stamp_seconds = rclcpp::Time(this->get_clock()->now()).seconds();
    double cloud_stamp_seconds = rclcpp::Time(input_pointcloud_msg_ptr->header.stamp).seconds();

    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/latency_ms", 1000.f * (now_stamp_seconds - cloud_stamp_seconds));
  }

  // Clear indexes
  std::fill(next_ring_index_.begin(), next_ring_index_.end(), 0);

  // Clear pointcloud buffer
  auto host_buffer = host_buffer_.get();
  for (std::size_t i = 0; i < num_rings_ * max_points_per_ring_; i++) {
    host_buffer[i] = autoware::point_types::PointXYZIRCAEDT{};
  }
}

void CudaPointcloudPreprocessorNode::cudaPointcloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> input_pointcloud_msg_ptr)
{
  double t1 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::high_resolution_clock::now().time_since_epoch())
                       .count();
  static_assert(
    sizeof(InputPointType) == sizeof(autoware::point_types::PointXYZIRCAEDT),
    "PointStruct and PointXYZIRCAEDT must have the same size");

  stop_watch_ptr_->toc("processing_time", true);

  // Make sure that the first twist is newer than the first point
  InputPointType first_point;
  cudaMemcpy(
    &first_point, input_pointcloud_msg_ptr->data.get(), sizeof(InputPointType),
    cudaMemcpyDeviceToHost);
  double first_point_stamp = input_pointcloud_msg_ptr->header.stamp.sec +
                             input_pointcloud_msg_ptr->header.stamp.nanosec * 1e-9 +
                             first_point.time_stamp * 1e-9;

  while (twist_queue_.size() > 1 &&
         rclcpp::Time(twist_queue_.front().header.stamp).seconds() < first_point_stamp) {
    twist_queue_.pop_front();
  }

  while (angular_velocity_queue_.size() > 1 &&
         rclcpp::Time(angular_velocity_queue_.front().header.stamp).seconds() < first_point_stamp) {
    angular_velocity_queue_.pop_front();
  }

  // Obtain the base link to input pointcloud transform
  geometry_msgs::msg::TransformStamped transform_msg;

  try {
    transform_msg = tf2_buffer_.lookupTransform(
      base_frame_, input_pointcloud_msg_ptr->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  double t2 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::high_resolution_clock::now().time_since_epoch())
                       .count();

  auto output_pointcloud_ptr = cuda_pointcloud_preprocessor_->process(
    *input_pointcloud_msg_ptr, transform_msg, twist_queue_, angular_velocity_queue_);
  output_pointcloud_ptr->header.frame_id = base_frame_;

  double t3 = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::high_resolution_clock::now().time_since_epoch())
                       .count();

  if (iteration_ >= 0) {
    processing_time_ms_ += (t2 - t1);
    kernel_time_ms_ += (t3 - t2);
  }
  iteration_++;

  if (num_rings_ > 64) {
    RCLCPP_INFO(
      get_logger(),
      "Organize time: %f ms, Copy time: %f ms, Process time: %f ms, Kernel time: %f ms Total time: "
      "%f ms",
      organize_time_ms_ / iteration_, copy_time_ms_ / iteration_, processing_time_ms_ / iteration_,
      kernel_time_ms_ / iteration_,
      (organize_time_ms_ + copy_time_ms_ + processing_time_ms_ + kernel_time_ms_) / iteration_);
    RCLCPP_INFO(
      get_logger(),
      "\t Cuda Organize time: %f ms, Cuda Copy time: %f ms, Cuda Process time: %f ms, Cuda Kernel "
      "time: %f ms Total time: %f ms",
      organize_time_cuda_ms_ / iteration_, copy_time_cuda_ms_ / iteration_,
      processing_time_ms_ / iteration_, kernel_time_ms_ / iteration_,
      (organize_time_cuda_ms_ + copy_time_cuda_ms_ + processing_time_ms_ + kernel_time_ms_) /
        iteration_);
  }

  // Publish
  pub_->publish(std::move(output_pointcloud_ptr));

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    double now_stamp_seconds = rclcpp::Time(this->get_clock()->now()).seconds();
    double cloud_stamp_seconds = rclcpp::Time(input_pointcloud_msg_ptr->header.stamp).seconds();

    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/latency_ms", 1000.f * (now_stamp_seconds - cloud_stamp_seconds));
  }

  // Preallocate for next iteration
  cuda_pointcloud_preprocessor_->preallocateOutput();
}

}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessorNode)
