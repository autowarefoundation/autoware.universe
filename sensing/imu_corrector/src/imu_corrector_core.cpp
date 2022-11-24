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

#include "imu_corrector/imu_corrector_core.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace imu_corrector
{
ImuCorrector::ImuCorrector(const rclcpp::NodeOptions & node_options)
: Node("imu_corrector", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  output_frame_(declare_parameter<std::string>("base_link", "base_link"))
{
  angular_velocity_offset_x_ = declare_parameter<double>("angular_velocity_offset_x", 0.0);
  angular_velocity_offset_y_ = declare_parameter<double>("angular_velocity_offset_y", 0.0);
  angular_velocity_offset_z_ = declare_parameter<double>("angular_velocity_offset_z", 0.0);

  angular_velocity_stddev_xx_ = declare_parameter<double>("angular_velocity_stddev_xx", 0.03);
  angular_velocity_stddev_yy_ = declare_parameter<double>("angular_velocity_stddev_yy", 0.03);
  angular_velocity_stddev_zz_ = declare_parameter<double>("angular_velocity_stddev_zz", 0.03);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "input", rclcpp::QoS{1}, std::bind(&ImuCorrector::callbackImu, this, std::placeholders::_1));

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
}

void ImuCorrector::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg = *imu_msg_ptr;

  imu_msg.angular_velocity.x -= angular_velocity_offset_x_;
  imu_msg.angular_velocity.y -= angular_velocity_offset_y_;
  imu_msg.angular_velocity.z -= angular_velocity_offset_z_;

  using IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;
  imu_msg.angular_velocity_covariance[IDX::X_X] =
    angular_velocity_stddev_xx_ * angular_velocity_stddev_xx_;
  imu_msg.angular_velocity_covariance[IDX::Y_Y] =
    angular_velocity_stddev_yy_ * angular_velocity_stddev_yy_;
  imu_msg.angular_velocity_covariance[IDX::Z_Z] =
    angular_velocity_stddev_zz_ * angular_velocity_stddev_zz_;

  geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  getTransform(output_frame_, imu_msg.header.frame_id, tf_base2imu_ptr);

  geometry_msgs::msg::Vector3Stamped acceleration;
  acceleration.header = imu_msg.header;
  acceleration.vector = imu_msg.linear_acceleration;

  geometry_msgs::msg::Vector3Stamped transformed_acceleration;
  transformed_acceleration.header = tf_base2imu_ptr->header;
  tf2::doTransform(acceleration, transformed_acceleration, *tf_base2imu_ptr);

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header = imu_msg.header;
  angular_velocity.vector = imu_msg.angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.header = tf_base2imu_ptr->header;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_base2imu_ptr);

  imu_msg.linear_acceleration = transformed_acceleration.vector;
  imu_msg.angular_velocity = transformed_angular_velocity.vector;
  imu_msg.header.frame_id = output_frame_;

  imu_pub_->publish(imu_msg);
}

bool ImuCorrector::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

}  // namespace imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_corrector::ImuCorrector)
