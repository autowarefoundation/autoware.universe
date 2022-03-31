// Copyright 2022 Tier IV, Inc.
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
//
//

#include "simple_object_tracker/simple_object_tracker_core.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <boost/optional.hpp>

#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

namespace
{
boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("simple_object_tracker"), ex.what());
    return boost::none;
  }
}

bool transformDetectedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_auto_perception_msgs::msg::DetectedObjects & output_msg)
{
  output_msg = input_msg;

  /* transform to world coordinate */
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (size_t i = 0; i < output_msg.objects.size(); ++i) {
      tf2::fromMsg(
        output_msg.objects.at(i).kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, output_msg.objects.at(i).kinematics.pose_with_covariance.pose);
      // TODO(yukkysaito) transform covariance
    }
  }
  return true;
}
}  // namespace

SimpleObjectTracker::SimpleObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("simple_object_tracker", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Create publishers and subscribers
  detected_object_sub_ = create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input", rclcpp::QoS{1},
    std::bind(&SimpleObjectTracker::onMeasurement, this, std::placeholders::_1));
  tracked_objects_pub_ =
    create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>("output", rclcpp::QoS{1});

  // Parameters
  world_frame_id_ = declare_parameter<std::string>("world_frame_id", "world");

  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);
}

void SimpleObjectTracker::onMeasurement(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objects_msg)
{
  const auto self_transform =
    getTransform(tf_buffer_, "base_link", world_frame_id_, input_objects_msg->header.stamp);
  if (!self_transform) {
    return;
  }

  /* transform to world coordinate */
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!transformDetectedObjects(
        *input_objects_msg, world_frame_id_, tf_buffer_, transformed_objects)) {
    return;
  }

  const auto subscriber_count = tracked_objects_pub_->get_subscription_count() +
                                tracked_objects_pub_->get_intra_process_subscription_count();
  if (subscriber_count < 1) {
    return;
  }

  // Create output msg
  autoware_auto_perception_msgs::msg::TrackedObjects output_msg;
  output_msg.header.frame_id = world_frame_id_;
  output_msg.header.stamp = input_objects_msg->header.stamp;
  for (const auto & input_object : input_objects_msg->objects) {
    autoware_auto_perception_msgs::msg::TrackedObject tracked_object;
    tracked_object.classification = input_object.classification;
    tracked_object.existence_probability = input_object.existence_probability;
    tracked_object.shape = input_object.shape;
    tracked_object.kinematics.pose_with_covariance = input_object.kinematics.pose_with_covariance;
    tracked_object.kinematics.twist_with_covariance = input_object.kinematics.twist_with_covariance;
    tracked_object.kinematics.orientation_availability =
      input_object.kinematics.orientation_availability;
    output_msg.objects.push_back(tracked_object);
  }

  tracked_objects_pub_->publish(output_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimpleObjectTracker)
