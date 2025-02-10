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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "debug_marker.hpp"
#include "surround_obstacle_checker_node_parameters.hpp"

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::surround_obstacle_checker
{

using autoware::motion_utils::VehicleStopChecker;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_planning_msgs::msg::VelocityLimitClearCommand;

using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode : public rclcpp::Node
{
public:
  explicit SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  std::array<double, 3> getCheckDistances(const std::string & str_label) const;

  bool getUseDynamicObject() const;

  void onTimer();

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void onDynamicObjects(const PredictedObjects::ConstSharedPtr msg);

  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::optional<Obstacle> getNearestObstacle() const;

  std::optional<Obstacle> getNearestObstacleByPointCloud() const;

  std::optional<Obstacle> getNearestObstacleByDynamicObject() const;

  std::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  auto isStopRequired(
    const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
    const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold) const
    -> std::pair<bool, std::optional<rclcpp::Time>>;

  // ros
  mutable tf2_ros::Buffer tf_buffer_{get_clock()};
  mutable tf2_ros::TransformListener tf_listener_{tf_buffer_};
  rclcpp::TimerBase::SharedPtr timer_;

  // publisher and subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odometry_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<sensor_msgs::msg::PointCloud2>
    sub_pointcloud_{this, "~/input/pointcloud", autoware::universe_utils::SingleDepthSensorQoS()};
  autoware::universe_utils::InterProcessPollingSubscriber<PredictedObjects> sub_dynamic_objects_{
    this, "~/input/objects"};
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;

  // parameter callback result
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // stop checker
  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;

  // debug
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;

  // parameter
  std::shared_ptr<surround_obstacle_checker_node::ParamListener> param_listener_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // data
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr_;
  PredictedObjects::ConstSharedPtr object_ptr_;

  // State Machine
  State state_ = State::PASS;
  std::optional<rclcpp::Time> last_obstacle_found_time_;

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unordered_map<int, std::string> label_map_;

public:
  friend class SurroundObstacleCheckerNodeTest;
};
}  // namespace autoware::surround_obstacle_checker

#endif  // NODE_HPP_
