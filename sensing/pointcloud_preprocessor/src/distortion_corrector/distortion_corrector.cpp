// Copyright 2020 Tier IV, Inc.
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

#include "pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include "pointcloud_preprocessor/distortion_corrector/undistorter.hpp"
#include "tier4_autoware_utils/math/trigonometry.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

// delete this
#include <chrono>
#include <deque>
#include <optional>
#include <string>
#include <utility>

namespace pointcloud_preprocessor
{
/** @brief Constructor. */
DistortionCorrectorComponent::DistortionCorrectorComponent(const rclcpp::NodeOptions & options)
: Node("distortion_corrector_node", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "distortion_corrector");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Parameter
  time_stamp_field_name_ = declare_parameter("time_stamp_field_name", "time_stamp");
  use_imu_ = declare_parameter("use_imu", true);
  use_3d_distortion_correction_ = declare_parameter("use_3d_distortion_correction", false);

  // Publisher
  undistorted_points_pub_ =
    this->create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS());

  // Subscriber
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(
      &DistortionCorrectorComponent::onTwistWithCovarianceStamped, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", 10,
    std::bind(&DistortionCorrectorComponent::onImu, this, std::placeholders::_1));
  input_points_sub_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DistortionCorrectorComponent::onPointCloud, this, std::placeholders::_1));

  // Setup the undistortor

  if (use_3d_distortion_correction_) {
    undistorter_ = std::make_unique<Undistorter3D>(tf2_buffer);
  } else {
    undistorter_ = std::make_unique<Undistorter2D>(tf2_buffer);
  }
}

void DistortionCorrectorComponent::onTwistWithCovarianceStamped(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist = twist_msg->twist.twist;
  twist_queue_.push_back(msg);

  while (!twist_queue_.empty()) {
    // for replay rosbag
    if (rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg->header.stamp)) {
      twist_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_queue_.front().header.stamp) <
      rclcpp::Time(twist_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue_.pop_front();
    }
    break;
  }
}

void DistortionCorrectorComponent::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  if (!use_imu_) {
    return;
  }

  if (!undistorter_->is_imu_transfrom_exist)
    undistorter_->setIMUTransform(base_link_frame_, imu_msg->header.frame_id);

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(
    angular_velocity, transformed_angular_velocity, *undistorter_->geometry_imu_to_base_link_ptr);
  transformed_angular_velocity.header = imu_msg->header;
  angular_velocity_queue_.push_back(transformed_angular_velocity);

  while (!angular_velocity_queue_.empty()) {
    // for replay rosbag
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

void DistortionCorrectorComponent::onPointCloud(PointCloud2::UniquePtr pointcloud_msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  const auto points_sub_count = undistorted_points_pub_->get_subscription_count() +
                                undistorted_points_pub_->get_intra_process_subscription_count();

  if (points_sub_count < 1) {
    return;
  }

  if (!undistorter_->is_pointcloud_transfrom_exist)
    undistorter_->setPointCloudTransform(base_link_frame_, pointcloud_msg->header.frame_id);

  undistorter_->initialize();
  undistortPointCloud(*pointcloud_msg);

  if (debug_publisher_) {
    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - pointcloud_msg->header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  undistorted_points_pub_->publish(std::move(pointcloud_msg));

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

bool DistortionCorrectorComponent::isInputValid(PointCloud2 & pointcloud)
{
  if (pointcloud.data.empty() || twist_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */, "input pointcloud or twist_queue_ is empty.");
    return false;
  }

  auto time_stamp_field_it = std::find_if(
    std::cbegin(pointcloud.fields), std::cend(pointcloud.fields),
    [this](const sensor_msgs::msg::PointField & field) {
      return field.name == time_stamp_field_name_;
    });
  if (time_stamp_field_it == pointcloud.fields.cend()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "Required field time stamp doesn't exist in the point cloud.");
    return false;
  }
  return true;
}

void DistortionCorrectorComponent::warnIfTimestampsTooLate(
  bool is_twist_time_stamp_too_late, bool is_imu_time_stamp_is_too_late)
{
  if (is_twist_time_stamp_too_late) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "twist time_stamp is too late. Could not interpolate.");
  }

  if (is_imu_time_stamp_is_too_late) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "imu time_stamp is too late. Could not interpolate.");
  }
}

void DistortionCorrectorComponent::getIteratorOfTwistAndIMU(
  double first_point_time_stamp_sec,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu)
{
  it_twist = std::lower_bound(
    std::begin(twist_queue_), std::end(twist_queue_), first_point_time_stamp_sec,
    [](const geometry_msgs::msg::TwistStamped & x, const double t) {
      return rclcpp::Time(x.header.stamp).seconds() < t;
    });
  it_twist = it_twist == std::end(twist_queue_) ? std::end(twist_queue_) - 1 : it_twist;

  if (use_imu_ && !angular_velocity_queue_.empty()) {
    it_imu = std::lower_bound(
      std::begin(angular_velocity_queue_), std::end(angular_velocity_queue_),
      first_point_time_stamp_sec, [](const geometry_msgs::msg::Vector3Stamped & x, const double t) {
        return rclcpp::Time(x.header.stamp).seconds() < t;
      });
    it_imu =
      it_imu == std::end(angular_velocity_queue_) ? std::end(angular_velocity_queue_) - 1 : it_imu;
  }
}

void DistortionCorrectorComponent::undistortPointCloud(PointCloud2 & pointcloud)
{
  if (!isInputValid(pointcloud)) return;

  sensor_msgs::PointCloud2Iterator<float> it_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(pointcloud, "z");
  sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(pointcloud, time_stamp_field_name_);

  double prev_time_stamp_sec{*it_time_stamp};
  const double first_point_time_stamp_sec{*it_time_stamp};

  std::deque<geometry_msgs::msg::TwistStamped>::iterator it_twist;
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator it_imu;
  getIteratorOfTwistAndIMU(first_point_time_stamp_sec, it_twist, it_imu);

  // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
  double twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
  double imu_stamp{0.0};
  if (use_imu_ && !angular_velocity_queue_.empty()) {
    imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
  }

  // If there is a point in a pointlcoud that cannot be associated, record it to issue a warning
  bool is_twist_time_stamp_too_late = false;
  bool is_imu_time_stamp_is_too_late = false;
  bool is_twist_valid = true;
  bool is_imu_valid = true;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {
    auto start = std::chrono::high_resolution_clock::now();
    is_twist_valid = true;
    is_imu_valid = true;

    // Get closest twist information
    while (it_twist != std::end(twist_queue_) - 1 && *it_time_stamp > twist_stamp) {
      ++it_twist;
      twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
    }
    if (std::abs(*it_time_stamp - twist_stamp) > 0.1) {
      is_twist_time_stamp_too_late = true;
      is_twist_valid = false;
    }

    // Get closest IMU information
    if (use_imu_ && !angular_velocity_queue_.empty()) {
      while (it_imu != std::end(angular_velocity_queue_) - 1 && *it_time_stamp > imu_stamp) {
        ++it_imu;
        imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
      }

      if (std::abs(*it_time_stamp - imu_stamp) > 0.1) {
        is_imu_time_stamp_is_too_late = true;
        is_imu_valid = false;
      }
    } else {
      is_imu_valid = false;
    }

    float time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);

    // std::cout << "before undistortPoint" << std::endl;
    // std::cout << "it_x: " << *it_x << " it_y: " << *it_y << " it_z: " << *it_z << std::endl;

    // Undistorted a single point based on the strategy
    undistorter_->undistortPoint(
      it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);

    // std::cout << "after undistortPoint" << std::endl;
    // std::cout << "it_x: " << *it_x << " it_y: " << *it_y << " it_z: " << *it_z << std::endl;
    // std::cout << "//////////////////\n" << std::endl;
    prev_time_stamp_sec = *it_time_stamp;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Function execution time: " << duration.count() << " seconds" << std::endl;
  }

  warnIfTimestampsTooLate(is_twist_time_stamp_too_late, is_imu_time_stamp_is_too_late);
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::DistortionCorrectorComponent)
