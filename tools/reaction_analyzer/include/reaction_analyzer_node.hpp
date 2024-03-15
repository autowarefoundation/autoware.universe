// Copyright 2024 The Autoware Contributors
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

#ifndef REACTION_ANALYZER_NODE_HPP_
#define REACTION_ANALYZER_NODE_HPP_

#include "tf2/transform_datatypes.h"

#include <pcl/impl/point_types.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <subscriber.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <topic_publisher.hpp>
#include <utils.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace reaction_analyzer
{
using autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_adapi_v1_msgs::msg::RouteState;
using autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

// The running mode of the node
enum class RunningMode {
  PerceptionPlanning = 0,
  PlanningControl = 1,
};

struct PoseParams
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct EntityParams
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  double x_l;
  double y_l;
  double z_l;
};

struct NodeParams
{
  std::string running_mode;
  double timer_period;
  std::string output_file_path;
  size_t test_iteration;
  double spawn_time_after_init;
  double spawn_distance_threshold;
  double spawned_pointcloud_sampling_distance;
  double dummy_perception_publisher_period;
  PoseParams initial_pose;
  PoseParams goal_pose;
  EntityParams entity_params;
};

class ReactionAnalyzerNode : public rclcpp::Node
{
public:
  explicit ReactionAnalyzerNode(rclcpp::NodeOptions options);
  ~ReactionAnalyzerNode() = default;

  Odometry::ConstSharedPtr odometry_ptr_;

private:
  std::mutex mutex_;
  RunningMode node_running_mode_;

  // Parameters
  NodeParams node_params_;

  // Initialization Variables
  geometry_msgs::msg::Pose entity_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  // Subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<LocalizationInitializationState>::SharedPtr sub_localization_init_state_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_ground_truth_pose_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_predicted_objects_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;

  // tf
  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // Variables
  std::unordered_map<std::string, std::vector<double>> test_results_;
  std::optional<rclcpp::Time> last_test_environment_init_request_time_;
  std::optional<rclcpp::Time> test_environment_init_time_;
  std::optional<rclcpp::Time> spawn_cmd_time_;
  std::atomic<bool> spawn_object_cmd_{false};
  std::atomic<bool> is_object_spawned_message_published_{false};
  bool is_vehicle_initialized_{false};
  bool is_route_set_{false};
  size_t test_iteration_count_{0};

  // Functions
  void initAnalyzerVariables();

  void initPointcloud();

  void initPredictedObjects();

  void initEgoForTest(
    const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
    const RouteState::ConstSharedPtr & route_state_ptr,
    const OperationModeState::ConstSharedPtr & operation_mode_ptr,
    const PoseStamped::ConstSharedPtr & ground_truth_pose_ptr,
    const Odometry::ConstSharedPtr & odometry_ptr);

  void callOperationModeServiceWithoutResponse();

  void spawnObstacle(const geometry_msgs::msg::Point & ego_pose);

  void calculateResults(
    const std::unordered_map<std::string, subscriber::BufferVariant> & message_buffers,
    const rclcpp::Time & spawn_cmd_time);

  void onTimer();

  void dummyPerceptionPublisher();

  void reset();

  void writeResultsToFile();

  // Callbacks
  void vehiclePoseCallback(Odometry::ConstSharedPtr msg_ptr);

  void initializationStateCallback(LocalizationInitializationState::ConstSharedPtr msg_ptr);

  void routeStateCallback(RouteState::ConstSharedPtr msg_ptr);

  void operationModeCallback(OperationModeState::ConstSharedPtr msg_ptr);

  void groundTruthPoseCallback(PoseStamped::ConstSharedPtr msg_ptr);
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr dummy_perception_timer_;

  // Client
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_autonomous_;

  // Pointers
  std::shared_ptr<topic_publisher::TopicPublisher> topic_publisher_ptr_;
  std::unique_ptr<subscriber::SubscriberBase> subscriber_ptr_;
  PointCloud2::SharedPtr entity_pointcloud_ptr_;
  PredictedObjects::SharedPtr predicted_objects_ptr_;
  LocalizationInitializationState::ConstSharedPtr initialization_state_ptr_;
  RouteState::ConstSharedPtr current_route_state_ptr_;
  OperationModeState::ConstSharedPtr operation_mode_ptr_;
  PoseStamped::ConstSharedPtr ground_truth_pose_ptr_;
};
}  // namespace reaction_analyzer

#endif  // REACTION_ANALYZER_NODE_HPP_
