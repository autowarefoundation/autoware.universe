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

#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <topic_publisher.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_internal_msgs/msg/published_time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
#include <utility>
#include <variant>

namespace reaction_analyzer
{
using autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_adapi_v1_msgs::msg::RouteState;
using autoware_adapi_v1_msgs::srv::ChangeOperationMode;
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

// The supported message types
enum class SubscriberMessageType {
  Unknown = 0,
  AckermannControlCommand = 1,
  Trajectory = 2,
  PointCloud2 = 3,
  DetectedObjects = 4,
  PredictedObjects = 5,
  TrackedObjects = 6,
};

// The running mode of the node
enum class RunningMode {
  PerceptionPlanning = 0,
  PlanningControl = 1,
};

// The configuration of the topic to be subscribed which are defined in reaction_chain
struct TopicConfig
{
  std::string node_name;
  std::string topic_address;
  std::string time_debug_topic_address;
  SubscriberMessageType message_type;
};

using ChainModules = std::vector<TopicConfig>;

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
  double published_time_expire_duration;
  std::string output_file_path;
  size_t test_iteration;
  double object_search_radius_offset;
  double spawn_time_after_init;
  double spawn_distance_threshold;
  double spawned_pointcloud_sampling_distance;
  double dummy_perception_publisher_period;
  bool debug_control_commands;
  double control_cmd_buffer_time_interval;
  double min_jerk_for_brake_cmd;
  size_t min_number_descending_order_control_cmd;
  PoseParams initial_pose;
  PoseParams goal_pose;
  EntityParams entity_params;
};

// class PublishedTimeSubscriber
//{
// public:
//   PublishedTimeSubscriber(const std::string & topic_name, rclcpp::Node * node) : node_(node)
//   {
//     // Initialize subscriber with cache
//     message_subscriber_ =
//       std::make_shared<message_filters::Subscriber<PublishedTime>>(node, topic_name);
//     cache_ = std::make_shared<message_filters::Cache<PublishedTime>>(
//       *message_subscriber_, 10);  // Cache size of 100 messages
//   }
//
//   std::optional<PublishedTime> getPublishedTime(const std_msgs::msg::Header & header)
//   {
//     for (const auto & msg : cache_->getInterval(rclcpp::Time(0), node_->now())) {
//       if (msg->header.stamp == header.stamp && msg->header.frame_id == header.frame_id) {
//         return *msg;
//       }
//     }
//     return std::nullopt;  // Return an empty optional if no match is found
//   }
//
//   std::optional<PublishedTime> getPublishedTime(const rclcpp::Time & stamp)
//   {
//     for (const auto & msg : cache_->getInterval(rclcpp::Time(0), node_->now())) {
//       if (msg->header.stamp == stamp) {
//         return *msg;
//       }
//     }
//     return std::nullopt;  // Return an empty optional if no match is found
//   }
//
// private:
//   std::shared_ptr<message_filters::Subscriber<PublishedTime>> message_subscriber_;
//   std::shared_ptr<message_filters::Cache<PublishedTime>> cache_;
//   rclcpp::Node * node_;
// };

class ReactionAnalyzerNode : public rclcpp::Node
{
public:
  explicit ReactionAnalyzerNode(rclcpp::NodeOptions options);

  ~ReactionAnalyzerNode() = default;

private:
  std::mutex mutex_;
  RunningMode node_running_mode_;

  // Parameters
  NodeParams node_params_;

  // Initialization Variables
  geometry_msgs::msg::Pose entity_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  double entity_search_radius_;

  // Subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<LocalizationInitializationState>::SharedPtr sub_localization_init_state_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_predicted_objects_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;

  // tf
  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // Variables
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
  std::unordered_map<std::string, BufferVariant> message_buffers_;
  std::unique_ptr<message_filters::Subscriber<PublishedTime>> time_debug_sub_;
  std::unordered_map<std::string, std::vector<PublishedTime>> published_time_vector_map_;
  //  std::unordered_map<std::string, PublishedTimeSubscriber> published_time_subscriber_map_;

  std::unordered_map<std::string, std::vector<double>> test_results_;

  std::optional<rclcpp::Time> last_test_environment_init_request_time_;
  std::optional<rclcpp::Time> test_environment_init_time_;
  std::optional<rclcpp::Time> spawn_cmd_time_;
  std::atomic<bool> spawn_object_cmd_{false};
  std::atomic<bool> is_object_spawned_message_published_{false};
  bool is_output_printed_{false};
  bool is_vehicle_initialized_{false};
  bool is_route_set_{false};
  size_t test_iteration_count_{0};

  // Functions
  rclcpp::SubscriptionOptions createSubscriptionOptions();

  bool searchPointcloudNearEntity(const pcl::PointCloud<pcl::PointXYZ> & pcl_pointcloud);

  bool searchPredictedObjectsNearEntity(const PredictedObjects & predicted_objects);

  bool searchDetectedObjectsNearEntity(const DetectedObjects & detected_objects);

  bool searchTrackedObjectsNearEntity(const TrackedObjects & tracked_objects);

  bool loadChainModules();

  bool initSubscribers(const reaction_analyzer::ChainModules & modules);

  void initAnalyzerVariables();

  void initPointcloud();

  void initPredictedObjects();

  void initEgoForTest(
    const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
    const RouteState::ConstSharedPtr & route_state_ptr,
    const OperationModeState::ConstSharedPtr & operation_mode_ptr);

  void pushPublishedTime(
    std::vector<PublishedTime> & published_time_vec, const PublishedTime & published_time);

  void setControlCommandToBuffer(
    std::vector<AckermannControlCommand> & buffer, const AckermannControlCommand & cmd);

  std::optional<size_t> findFirstBrakeIdx(
    const std::vector<AckermannControlCommand> & cmd_array,
    const std::optional<rclcpp::Time> & spawn_cmd_time);

  void spawnObstacle(const geometry_msgs::msg::Point & ego_pose);

  void calculateResults(
    const std::unordered_map<std::string, BufferVariant> & message_buffers,
    const std::unordered_map<std::string, std::vector<PublishedTime>> & published_time_vector_map,
    const rclcpp::Time & spawn_cmd_time);

  void onTimer();

  bool allReacted(const std::unordered_map<std::string, BufferVariant> & message_buffers);

  void dummyPerceptionPublisher();

  void reset();

  void writeResultsToFile();

  // Callbacks
  void vehiclePoseCallback(Odometry::ConstSharedPtr msg_ptr);

  void initializationStateCallback(LocalizationInitializationState::ConstSharedPtr msg_ptr);

  void routeStateCallback(RouteState::ConstSharedPtr msg);

  void operationModeCallback(OperationModeState::ConstSharedPtr msg_ptr);

  // Callbacks for modules are subscribed
  void controlCommandOutputCallback(
    const std::string & node_name, const AckermannControlCommand::ConstSharedPtr & msg_ptr);

  //  void controlCommandOutputCallback(
  //    const std::string & node_name, const AckermannControlCommand::ConstSharedPtr & msg_ptr,
  //    const PublishedTime::ConstSharedPtr & published_time_ptr);

  void trajectoryOutputCallback(
    const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr);

  //  void trajectoryOutputCallback(
  //    const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr,
  //    const PublishedTime::ConstSharedPtr & published_time_ptr);

  void pointcloud2OutputCallback(
    const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr);

  //  void pointcloud2OutputCallback(
  //    const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr,
  //    const PublishedTime::ConstSharedPtr & published_time_ptr);

  void predictedObjectsOutputCallback(
    const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr);

  //  void predictedObjectsOutputCallback(
  //    const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr,
  //    const PublishedTime::ConstSharedPtr & published_time_ptr);

  void detectedObjectsOutputCallback(
    const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr);

  //  void detectedObjectsOutputCallback(
  //    const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr,
  //    const PublishedTime::ConstSharedPtr & published_time_ptr);

  void trackedObjectsOutputCallback(
    const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr);

  //  void trackedObjectsOutputCallback(
  //    const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr,
  //    const PublishedTime::ConstSharedPtr & published_time_ptr);

  void publishedTimeOutputCallback(
    const std::string & node_name, const PublishedTime::ConstSharedPtr & msg_ptr);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr dummy_perception_timer_;

  // Client
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_autonomous_;

  void callOperationModeServiceWithoutResponse();

  // Pointers
  std::shared_ptr<topic_publisher::TopicPublisher> topic_publisher_ptr_;
  PointCloud2::SharedPtr entity_pointcloud_ptr_;
  PredictedObjects::SharedPtr predicted_objects_ptr_;
  Odometry::ConstSharedPtr odometry_;
  LocalizationInitializationState::ConstSharedPtr initialization_state_ptr_;
  RouteState::ConstSharedPtr current_route_state_ptr_;
  OperationModeState::ConstSharedPtr operation_mode_ptr_;
};

}  // namespace reaction_analyzer

#endif  // REACTION_ANALYZER_NODE_HPP_
