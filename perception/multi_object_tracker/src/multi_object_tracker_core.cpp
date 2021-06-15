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
//
//
// Author: v1.0 Yukihiro Saito
//

#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/create_timer_interface.h"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "multi_object_tracker/multi_object_tracker_core.hpp"
#include "multi_object_tracker/utils/utils.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using SemanticType = autoware_perception_msgs::msg::Semantic;

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
  enable_delay_compensation_ = declare_parameter<bool>("enable_delay_compensation", false);

  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);

  // Create ROS time based timer
  auto timer_callback = std::bind(&MultiObjectTracker::publishTimerCallback, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate));

  publish_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(publish_timer_, nullptr);

  this->declare_parameter("can_assign_matrix");
  this->declare_parameter("max_dist_matrix");
  this->declare_parameter("max_area_matrix");
  this->declare_parameter("min_area_matrix");
  auto can_assign_matrix_tmp =
    this->get_parameter("can_assign_matrix").as_integer_array();
  std::vector<int> can_assign_matrix(can_assign_matrix_tmp.begin(), can_assign_matrix_tmp.end());
  std::vector<double> max_dist_matrix =
    this->get_parameter("max_dist_matrix").as_double_array();
  std::vector<double> max_area_matrix =
    this->get_parameter("max_area_matrix").as_double_array();
  std::vector<double> min_area_matrix =
    this->get_parameter("min_area_matrix").as_double_array();
  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_area_matrix, min_area_matrix);
}

void MultiObjectTracker::measurementCallback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr
  input_objects_msg)
{
  /* transform to world coordinate */
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray input_transformed_objects;
  if (!MultiObjectTracker::transformDynamicObjects(
      *input_objects_msg, world_frame_id_, input_transformed_objects))
  {
    return;
  }
  /* tracker prediction */
  rclcpp::Time measurement_time = input_objects_msg->header.stamp;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(measurement_time);
  }

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  Eigen::MatrixXd score_matrix = data_association_->calcScoreMatrix(
    input_transformed_objects, list_tracker_);  // row : tracker, col : measurement
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  /* tracker measurement update */
  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
    ++tracker_itr, ++tracker_idx)
  {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {  // found
      (*(tracker_itr))
      ->updateWithMeasurement(
        input_transformed_objects.feature_objects.at(direct_assignment.find(tracker_idx)->second)
        .object,
        measurement_time);
    } else {  // not found
      (*(tracker_itr))->updateWithoutMeasurement();
    }
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, measurement_time);

  /* new tracker */
  for (size_t i = 0; i < input_transformed_objects.feature_objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    const int & type = input_transformed_objects.feature_objects.at(i).object.semantic.type;
    if (type == SemanticType::CAR || type == SemanticType::TRUCK || type == SemanticType::BUS) {
      list_tracker_.push_back(
        std::make_shared<MultipleVehicleTracker>(
          measurement_time, input_transformed_objects.feature_objects.at(i).object));
    } else if (type == SemanticType::PEDESTRIAN) {
      list_tracker_.push_back(
        std::make_shared<PedestrianAndBicycleTracker>(
          measurement_time, input_transformed_objects.feature_objects.at(i).object));
    } else if (type == SemanticType::BICYCLE || type == SemanticType::MOTORBIKE) {
      list_tracker_.push_back(
        std::make_shared<PedestrianAndBicycleTracker>(
          measurement_time, input_transformed_objects.feature_objects.at(i).object));
    } else {
      list_tracker_.push_back(
        std::make_shared<UnknownTracker>(
          measurement_time, input_transformed_objects.feature_objects.at(i).object));
    }
  }

  if (!enable_delay_compensation_) {
    publish(measurement_time);
  }
}

void MultiObjectTracker::publishTimerCallback()
{
  if (enable_delay_compensation_) {
    rclcpp::Time current_time = this->now();
    /* life cycle check */
    checkTrackerLifeCycle(list_tracker_, current_time);

    // Publish
    publish(current_time);
    return;
  }
}

bool MultiObjectTracker::transformDynamicObjects(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & input_msg,
  const std::string & target_frame_id,
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & output_msg)
{
  output_msg = input_msg;

  /* transform to world coordinate */
  if (input_msg.header.frame_id != target_frame_id) {
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    try {
      geometry_msgs::msg::TransformStamped ros_target2objects_world;
      ros_target2objects_world = tf_buffer_.lookupTransform(
        /*target*/ target_frame_id, /*src*/ input_msg.header.frame_id, input_msg.header.stamp,
        rclcpp::Duration::from_seconds(0.5));
      tf2::fromMsg(ros_target2objects_world.transform, tf_target2objects_world);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return false;
    }
    for (size_t i = 0; i < output_msg.feature_objects.size(); ++i) {
      tf2::fromMsg(
        output_msg.feature_objects.at(i).object.state.pose_covariance.pose,
        tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(
        tf_target2objects, output_msg.feature_objects.at(i).object.state.pose_covariance.pose);
    }
  }
  return true;
}

void MultiObjectTracker::checkTrackerLifeCycle(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time)
{
  /* delete old tracker */
  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    if (1.0 < (*itr)->getElapsedTimeFromLastUpdate(this->now())) {
      auto erase_itr = itr;
      --itr;
      list_tracker.erase(erase_itr);
    }
  }

  /* delete collision tracker */
  for (auto itr1 = list_tracker.begin(); itr1 != list_tracker.end(); ++itr1) {
    autoware_perception_msgs::msg::DynamicObject object1;
    (*itr1)->getEstimatedDynamicObject(time, object1);
    for (auto itr2 = std::next(itr1); itr2 != list_tracker.end(); ++itr2) {
      autoware_perception_msgs::msg::DynamicObject object2;
      (*itr2)->getEstimatedDynamicObject(time, object2);
      constexpr double distance_threshold = 5.0f;
      const double distance = std::hypot(
        object1.state.pose_covariance.pose.position.x -
        object2.state.pose_covariance.pose.position.x,
        object1.state.pose_covariance.pose.position.y -
        object2.state.pose_covariance.pose.position.y);
      if (distance_threshold < distance) {continue;}
      if (0.1 < utils::get2dIoU(object1, object2)) {
        if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
          itr1 = list_tracker.erase(itr1);
          --itr1;
          break;
        } else {
          itr2 = list_tracker.erase(itr2);
          --itr2;
        }
      }
    }
  }
}

void MultiObjectTracker::publish(const rclcpp::Time & time)
{
  const auto subscriber_count = dynamic_object_pub_->get_subscription_count() +
    dynamic_object_pub_->get_intra_process_subscription_count();
  if (subscriber_count < 1) {return;}
  // Create output msg
  autoware_perception_msgs::msg::DynamicObjectArray output_msg;
  output_msg.header.frame_id = world_frame_id_;
  output_msg.header.stamp = time;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if ((*itr)->getTotalMeasurementCount() < 3) {continue;}
    autoware_perception_msgs::msg::DynamicObject object;
    (*itr)->getEstimatedDynamicObject(time, object);
    output_msg.objects.push_back(object);
  }

  // Publish
  dynamic_object_pub_->publish(output_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(MultiObjectTracker)
