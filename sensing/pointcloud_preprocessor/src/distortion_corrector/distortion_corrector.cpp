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

#include <deque>
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

  // Publisher
  undistorted_points_pub_ =
    this->create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS());

  // Subscriber
  input_twist_with_covariance_sub_ = this->create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist_with_covariance", 10,
    std::bind(&DistortionCorrectorComponent::onTwistWithCovariance, this, std::placeholders::_1));
  input_points_sub_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DistortionCorrectorComponent::onPointCloud, this, std::placeholders::_1));
}

void DistortionCorrectorComponent::onTwistWithCovariance(
  const TwistWithCovarianceStamped::ConstSharedPtr twist_with_covariance_msg)
{
  twist_with_covariance_queue_.push_back(*twist_with_covariance_msg);

  while (!twist_with_covariance_queue_.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(twist_with_covariance_queue_.front().header.stamp) >
      rclcpp::Time(twist_with_covariance_msg->header.stamp)) {
      twist_with_covariance_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_with_covariance_queue_.front().header.stamp) <
      rclcpp::Time(twist_with_covariance_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_with_covariance_queue_.pop_front();
    }
    break;
  }
}

void DistortionCorrectorComponent::onPointCloud(PointCloud2::UniquePtr points_msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  const auto points_sub_count = undistorted_points_pub_->get_subscription_count() +
                                undistorted_points_pub_->get_intra_process_subscription_count();

  if (points_sub_count < 1) {
    return;
  }

  tf2::Transform tf2_base_link_to_sensor{};
  getTransform(points_msg->header.frame_id, base_link_frame_, &tf2_base_link_to_sensor);

  undistortPointCloud(twist_with_covariance_queue_, tf2_base_link_to_sensor, *points_msg);

  if (points_sub_count > 0) {
    undistorted_points_pub_->publish(std::move(points_msg));
  }
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

bool DistortionCorrectorComponent::getTransform(
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

bool DistortionCorrectorComponent::undistortPointCloud(
  const std::deque<TwistWithCovarianceStamped> & twist_with_covariance_queue_,
  const tf2::Transform & tf2_base_link_to_sensor, PointCloud2 & points)
{
  if (points.data.empty() || twist_with_covariance_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "input_pointcloud->points or twist_with_covariance_queue_ is empty.");
    return false;
  }

  auto time_stamp_field_it = std::find_if(
    std::cbegin(points.fields), std::cend(points.fields),
    [this](const sensor_msgs::msg::PointField & field) {
      return field.name == time_stamp_field_name_;
    });
  if (time_stamp_field_it == points.fields.cend()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "Required field time stamp doesn't exist in the point cloud.");
    return false;
  }

  sensor_msgs::PointCloud2Iterator<float> it_x(points, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(points, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(points, "z");
  sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(points, time_stamp_field_name_);

  float theta{0.0f};
  float x{0.0f};
  float y{0.0f};
  double prev_time_stamp_sec{*it_time_stamp};
  const double first_point_time_stamp_sec{*it_time_stamp};

  auto twist_with_covariance_it = std::lower_bound(
    std::begin(twist_with_covariance_queue_), std::end(twist_with_covariance_queue_),
    first_point_time_stamp_sec, [](const TwistWithCovarianceStamped & x, const double t) {
      return rclcpp::Time(x.header.stamp).seconds() < t;
    });
  twist_with_covariance_it = twist_with_covariance_it == std::end(twist_with_covariance_queue_)
                               ? std::end(twist_with_covariance_queue_) - 1
                               : twist_with_covariance_it;

  const tf2::Transform tf2_base_link_to_sensor_inv{tf2_base_link_to_sensor.inverse()};
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {
    for (;
         (twist_with_covariance_it != std::end(twist_with_covariance_queue_) - 1 &&
          *it_time_stamp > rclcpp::Time(twist_with_covariance_it->header.stamp).seconds());
         ++twist_with_covariance_it) {
    }

    float v{static_cast<float>(twist_with_covariance_it->twist.twist.linear.x)};
    float w{static_cast<float>(twist_with_covariance_it->twist.twist.angular.z)};

    if (
      std::abs(*it_time_stamp - rclcpp::Time(twist_with_covariance_it->header.stamp).seconds()) >
      0.1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 10000 /* ms */,
        "twist_with_covariance time_stamp is too late. Cloud not interpolate.");
      v = 0.0f;
      w = 0.0f;
    }

    const float time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);

    const tf2::Vector3 sensorTF_point{*it_x, *it_y, *it_z};

    const tf2::Vector3 base_linkTF_point{tf2_base_link_to_sensor_inv * sensorTF_point};

    theta += w * time_offset;
    tf2::Quaternion baselink_quat{};
    baselink_quat.setRPY(0.0, 0.0, theta);
    const float dis = v * time_offset;
    x += dis * std::cos(theta);
    y += dis * std::sin(theta);

    tf2::Transform baselinkTF_odom{};
    baselinkTF_odom.setOrigin(tf2::Vector3(x, y, 0.0));
    baselinkTF_odom.setRotation(baselink_quat);

    const tf2::Vector3 base_linkTF_trans_point{baselinkTF_odom * base_linkTF_point};

    const tf2::Vector3 sensorTF_trans_point{tf2_base_link_to_sensor * base_linkTF_trans_point};

    *it_x = sensorTF_trans_point.getX();
    *it_y = sensorTF_trans_point.getY();
    *it_z = sensorTF_trans_point.getZ();

    prev_time_stamp_sec = *it_time_stamp;
  }
  return true;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::DistortionCorrectorComponent)
