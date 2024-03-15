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

#ifndef SUBSCRIBER_HPP_
#define SUBSCRIBER_HPP_
#include <motion_utils/trajectory/trajectory.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <utils.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_internal_msgs/msg/published_time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
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

namespace reaction_analyzer::subscriber
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_internal_msgs::msg::PublishedTime;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

// Buffers to be used to store subscribed messages
using ControlCommandBuffer =
  std::pair<std::vector<AckermannControlCommand>, std::optional<AckermannControlCommand>>;
using TrajectoryBuffer = std::optional<Trajectory>;
using PointCloud2Buffer = std::optional<PointCloud2>;
using PredictedObjectsBuffer = std::optional<PredictedObjects>;
using DetectedObjectsBuffer = std::optional<DetectedObjects>;
using TrackedObjectsBuffer = std::optional<TrackedObjects>;

// Variant to store different types of buffers
using BufferVariant = std::variant<
  ControlCommandBuffer, TrajectoryBuffer, PointCloud2Buffer, PredictedObjectsBuffer,
  DetectedObjectsBuffer, TrackedObjectsBuffer>;

template <typename MessageType>
struct SubscriberVariables
{
  using ExactTimePolicy = message_filters::sync_policies::ExactTime<MessageType, PublishedTime>;

  std::unique_ptr<message_filters::Subscriber<MessageType>> sub1_;
  std::unique_ptr<message_filters::Subscriber<PublishedTime>> sub2_;
  std::unique_ptr<message_filters::Synchronizer<ExactTimePolicy>> synchronizer_;
  // tmp: only for the messages who don't have header e.g. AckermannControlCommand
  std::unique_ptr<message_filters::Cache<PublishedTime>> cache_;
};

// Variant to create subscribers for different message types
using SubscriberVariablesVariant = std::variant<
  SubscriberVariables<PointCloud2>, SubscriberVariables<DetectedObjects>,
  SubscriberVariables<TrackedObjects>, SubscriberVariables<PredictedObjects>,
  SubscriberVariables<Trajectory>, SubscriberVariables<AckermannControlCommand>>;

// The supported message types
enum class SubscriberMessageType {
  UNKNOWN = 0,
  ACKERMANN_CONTROL_COMMAND = 1,
  TRAJECTORY = 2,
  POINTCLOUD2 = 3,
  DETECTED_OBJECTS = 4,
  PREDICTED_OBJECTS = 5,
  TRACKED_OBJECTS = 6,
};

// Reaction Types
enum class ReactionType {
  UNKNOWN = 0,
  FIRST_BRAKE = 1,
  SEARCH_ZERO_VEL = 2,
  SEARCH_ENTITY = 3,
};

// The configuration of the topic to be subscribed which are defined in reaction_chain
struct TopicConfig
{
  std::string node_name;
  std::string topic_address;
  std::string time_debug_topic_address;
  SubscriberMessageType message_type;
};

// Place for the reaction functions' parameter configuration
struct FirstBrakeParams
{
  bool debug_control_commands;
  double control_cmd_buffer_time_interval;
  size_t min_number_descending_order_control_cmd;
  double min_jerk_for_brake_cmd;
};

struct SearchZeroVelParams
{
  double max_looking_distance;
};

struct SearchEntityParams
{
  double search_radius;
};

// Place for the store the reaction parameter configuration (currently only for first brake)
struct ReactionParams
{
  FirstBrakeParams first_brake_params;
  SearchZeroVelParams search_zero_vel_params;
  SearchEntityParams search_entity_params;
};

using ChainModules = std::vector<TopicConfig>;

class SubscriberBase
{
public:
  explicit SubscriberBase(
    rclcpp::Node * node, Odometry::ConstSharedPtr & odometry, geometry_msgs::msg::Pose entity_pose,
    std::atomic<bool> & spawn_object_cmd);

  ~SubscriberBase() = default;

  std::optional<std::unordered_map<std::string, BufferVariant>> getMessageBuffersMap();
  void reset();

private:
  std::mutex mutex_;

  rclcpp::Node * node_;
  Odometry::ConstSharedPtr odometry_;
  geometry_msgs::msg::Pose entity_pose_;
  std::atomic<bool> & spawn_object_cmd_;

  // Variables to be initialized in constructor
  ChainModules chain_modules_;
  ReactionParams reaction_params_{};

  // Variants
  std::unordered_map<std::string, SubscriberVariablesVariant> subscriber_variables_map_;
  std::unordered_map<std::string, BufferVariant> message_buffers_;

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Functions
  void init_reaction_chains_and_params();
  bool init_subscribers();
  bool search_pointcloud_near_entity(const pcl::PointCloud<pcl::PointXYZ> & pcl_pointcloud);
  bool search_predicted_objects_near_entity(const PredictedObjects & predicted_objects);
  bool search_detected_objects_near_entity(const DetectedObjects & detected_objects);
  bool search_tracked_objects_near_entity(const TrackedObjects & tracked_objects);
  void set_control_command_to_buffer(
    std::vector<AckermannControlCommand> & buffer, const AckermannControlCommand & cmd);
  std::optional<size_t> find_first_brake_idx(
    const std::vector<AckermannControlCommand> & cmd_array);

  // Callbacks for modules are subscribed
  void on_control_command(
    const std::string & node_name, const AckermannControlCommand::ConstSharedPtr & msg_ptr);
  void on_trajectory(const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr);
  void on_trajectory(
    const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_pointcloud(const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr);
  void on_pointcloud(
    const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_predicted_objects(
    const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr);
  void on_predicted_objects(
    const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_detected_objects(
    const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr);
  void on_detected_objects(
    const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
  void on_tracked_objects(
    const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr);
  void on_tracked_objects(
    const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr,
    const PublishedTime::ConstSharedPtr & published_time_ptr);
};

}  // namespace reaction_analyzer::subscriber

#endif  // SUBSCRIBER_HPP_
