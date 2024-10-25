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

#ifndef AUTOWARE__COLLISION_DETECTOR__NODE_HPP_
#define AUTOWARE__COLLISION_DETECTOR__NODE_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <utility>

namespace autoware::collision_detector
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;

using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

class CollisionDetectorNode : public rclcpp::Node
{
public:
  explicit CollisionDetectorNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    bool use_pointcloud;
    bool use_dynamic_object;
    double collision_distance;
  };

private:
  void checkCollision(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void onDynamicObjects(const PredictedObjects::ConstSharedPtr msg);

  void onOperationMode(const OperationModeState::ConstSharedPtr msg);

  boost::optional<Obstacle> getNearestObstacle() const;

  boost::optional<Obstacle> getNearestObstacleByPointCloud() const;

  boost::optional<Obstacle> getNearestObstacleByDynamicObject() const;

  boost::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  // ros
  mutable tf2_ros::Buffer tf_buffer_{get_clock()};
  mutable tf2_ros::TransformListener tf_listener_{tf_buffer_};
  rclcpp::TimerBase::SharedPtr timer_;

  // publisher and subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_dynamic_objects_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;

  // parameter
  NodeParam node_param_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // data
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr_;
  PredictedObjects::ConstSharedPtr object_ptr_;
  OperationModeState::ConstSharedPtr operation_mode_ptr_;
  rclcpp::Time last_obstacle_found_stamp_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;
};
}  // namespace autoware::collision_detector

#endif  // AUTOWARE__COLLISION_DETECTOR__NODE_HPP_
