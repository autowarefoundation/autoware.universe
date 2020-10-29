/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "multi_object_tracker/multi_object_tracker_core.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/create_timer_interface.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Create publishers and subscribers
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  dynamic_object_sub_ =
    create_subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
      "input", durable_qos,
      std::bind(&MultiObjectTracker::measurementCallback, this, std::placeholders::_1));
  dynamic_object_pub_ =
    create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>("output", rclcpp::QoS{1});

  // Parameters
  double publish_rate = declare_parameter<double>("publish_rate", 30.0);
  world_frame_id_ = declare_parameter<std::string>("world_frame_id", "world");

  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);

  // Create ROS time based timer
  auto timer_callback = std::bind(&MultiObjectTracker::publishTimerCallback, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(publish_rate));

  publish_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(publish_timer_, nullptr);
}

void MultiObjectTracker::measurementCallback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr
    input_objects_msg)
{
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray input_transformed_objects =
    *input_objects_msg;

  /* transform to world coordinate */
  if (input_objects_msg->header.frame_id != world_frame_id_) {
    tf2::Transform tf_world2objects_world;
    tf2::Transform tf_world2objects;
    tf2::Transform tf_objects_world2objects;
    geometry_msgs::msg::TransformStamped ros_world2objects_world;
    // Create the future object
    auto future = tf_buffer_.waitForTransform(
      world_frame_id_, input_transformed_objects.header.frame_id,
      input_transformed_objects.header.stamp, tf2::durationFromSec(0.0), [](auto &) {});

    // Check for transform ready status
    auto status = future.wait_for(tf2::durationFromSec(0.5));
    if (status != std::future_status::ready) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *(this->get_clock()), 1.0, "cannot transform to world frame form %s",
        input_transformed_objects.header.frame_id);
      return;
    }

    // Get and process the transform
    ros_world2objects_world = future.get();
    tf2::fromMsg(ros_world2objects_world.transform, tf_world2objects_world);

    for (size_t i = 0; i < input_transformed_objects.feature_objects.size(); ++i) {
      tf2::fromMsg(
        input_transformed_objects.feature_objects.at(i).object.state.pose_covariance.pose,
        tf_objects_world2objects);
      tf_world2objects = tf_world2objects_world * tf_objects_world2objects;
      tf2::toMsg(
        tf_world2objects,
        input_transformed_objects.feature_objects.at(i).object.state.pose_covariance.pose);
    }
  }

  /* tracker prediction */
  rclcpp::Time measurement_time = input_objects_msg->header.stamp;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(measurement_time);
  }

  /* life cycle check */
  // TODO

  /* global nearest neighboor */
  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;
  Eigen::MatrixXd score_matrix = data_association_.calcScoreMatrix(
    input_transformed_objects, list_tracker_);  // row : tracker, col : measurement
  data_association_.assign(score_matrix, direct_assignment, reverse_assignment);

  /* tracker measurement update */
  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end())  // found
    {
      (*(tracker_itr))
        ->updateWithMeasurement(
          input_transformed_objects.feature_objects.at(direct_assignment.find(tracker_idx)->second)
            .object,
          measurement_time);
    } else  // not found
    {
      (*(tracker_itr))->updateWithoutMeasurement();
    }
  }

  /* new tracker */
  for (size_t i = 0; i < input_transformed_objects.feature_objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end())  // found
      continue;

    if (
      input_transformed_objects.feature_objects.at(i).object.semantic.type ==
        autoware_perception_msgs::msg::Semantic::CAR ||
      input_transformed_objects.feature_objects.at(i).object.semantic.type ==
        autoware_perception_msgs::msg::Semantic::TRUCK ||
      input_transformed_objects.feature_objects.at(i).object.semantic.type ==
        autoware_perception_msgs::msg::Semantic::BUS) {
      list_tracker_.push_back(std::make_shared<VehicleTracker>(
        measurement_time, input_transformed_objects.feature_objects.at(i).object));
    } else if (
      input_transformed_objects.feature_objects.at(i).object.semantic.type ==
      autoware_perception_msgs::msg::Semantic::PEDESTRIAN) {
      list_tracker_.push_back(std::make_shared<PedestrianTracker>(
        measurement_time, input_transformed_objects.feature_objects.at(i).object));
    } else if (
      input_transformed_objects.feature_objects.at(i).object.semantic.type ==
        autoware_perception_msgs::msg::Semantic::BICYCLE ||
      input_transformed_objects.feature_objects.at(i).object.semantic.type ==
        autoware_perception_msgs::msg::Semantic::MOTORBIKE) {
      list_tracker_.push_back(std::make_shared<BicycleTracker>(
        measurement_time, input_transformed_objects.feature_objects.at(i).object));
    } else {
      // list_tracker_.push_back(std::make_shared<PedestrianTracker>(input_transformed_objects.feature_objects.at(i).object));
    }
  }
}

void MultiObjectTracker::publishTimerCallback()
{
  // Guard
  if (dynamic_object_pub_->get_subscription_count() < 1) return;

  /* life cycle check */
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if (1.0 < (*itr)->getElapsedTimeFromLastUpdate(rclcpp::Node::now())) {
      auto erase_itr = itr;
      --itr;
      list_tracker_.erase(erase_itr);
    }
  }

  // Create output msg
  rclcpp::Time current_time = rclcpp::Node::now();
  autoware_perception_msgs::msg::DynamicObjectArray output_msg;
  output_msg.header.frame_id = world_frame_id_;
  output_msg.header.stamp = current_time;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if ((*itr)->getTotalMeasurementCount() < 3) continue;
    autoware_perception_msgs::msg::DynamicObject object;
    (*itr)->getEstimatedDynamicObject(current_time, object);
    output_msg.objects.push_back(object);
  }

  // Publish
  dynamic_object_pub_->publish(output_msg);
  return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(MultiObjectTracker)
