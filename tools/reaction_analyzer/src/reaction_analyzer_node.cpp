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

#include "reaction_analyzer_node.hpp"

#include "tf2/transform_datatypes.h"

#include <pcl/impl/point_types.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <message_filters/sync_policies/approximate_time.h>

#include <algorithm>
#include <memory>

namespace reaction_analyzer
{

// Callbacks

void ReactionAnalyzerNode::publishedTimeOutputCallback(
  const std::string & node_name, const PublishedTime::ConstSharedPtr & msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & published_time_vector = published_time_vector_map_[node_name];
  pushPublishedTime(published_time_vector, *msg_ptr);
}

void ReactionAnalyzerNode::controlCommandOutputCallback(
  const std::string & node_name, const AckermannControlCommand::ConstSharedPtr & msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & variant = message_buffers_[node_name];
  const auto is_spawned = spawn_cmd_time_;
  if (!std::holds_alternative<ControlCommandBuffer>(variant)) {
    ControlCommandBuffer buffer(std::vector<AckermannControlCommand>{*msg_ptr}, std::nullopt);
    variant = buffer;
  }

  if (std::get<ControlCommandBuffer>(variant).second) {
    // reacted
    return;
  }
  setControlCommandToBuffer(std::get<ControlCommandBuffer>(variant).first, *msg_ptr);
  const auto brake_idx =
    findFirstBrakeIdx(std::get<ControlCommandBuffer>(variant).first, is_spawned);
  if (brake_idx) {
    std::get<ControlCommandBuffer>(variant).second =
      std::get<ControlCommandBuffer>(variant).first.at(brake_idx.value());
    RCLCPP_INFO(this->get_logger(), "Reacted %s", node_name.c_str());
  }
}

void ReactionAnalyzerNode::trajectoryOutputCallback(
  const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  const auto current_odometry_ptr = odometry_;
  const auto is_spawned = spawn_cmd_time_;
  if (!std::holds_alternative<TrajectoryBuffer>(variant)) {
    TrajectoryBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (!current_odometry_ptr || !is_spawned || std::get<TrajectoryBuffer>(variant).has_value()) {
    return;
  }

  const auto nearest_seg_idx = motion_utils::findNearestSegmentIndex(
    msg_ptr->points, current_odometry_ptr->pose.pose.position);

  const auto nearest_objects_seg_idx =
    motion_utils::findNearestIndex(msg_ptr->points, entity_pose_.position);

  const auto zero_vel_idx = motion_utils::searchZeroVelocityIndex(
    msg_ptr->points, nearest_seg_idx, nearest_objects_seg_idx);

  if (zero_vel_idx) {
    std::get<TrajectoryBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(this->get_logger(), "Reacted %s", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void ReactionAnalyzerNode::pointcloud2OutputCallback(
  const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  const auto is_spawned = spawn_cmd_time_;
  if (!std::holds_alternative<PointCloud2Buffer>(variant)) {
    PointCloud2Buffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (!is_spawned || std::get<PointCloud2Buffer>(variant).has_value()) {
    return;
  }

  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "map", msg_ptr->header.frame_id, this->now(), rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to look up transform from " << msg_ptr->header.frame_id << " to map");
    return;
  }

  // transform by using eigen matrix
  PointCloud2 transformed_points{};
  const Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(affine_matrix, *msg_ptr, transformed_points);

  pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
  pcl::fromROSMsg(transformed_points, pcl_pointcloud);

  if (searchPointcloudNearEntity(pcl_pointcloud)) {
    std::get<PointCloud2Buffer>(variant) = *msg_ptr;
    RCLCPP_INFO(this->get_logger(), "Reacted %s", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void ReactionAnalyzerNode::predictedObjectsOutputCallback(
  const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  const auto is_spawned = spawn_cmd_time_;
  if (!std::holds_alternative<PredictedObjectsBuffer>(variant)) {
    PredictedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (
    !is_spawned || msg_ptr->objects.empty() ||
    std::get<PredictedObjectsBuffer>(variant).has_value()) {
    return;
  }

  if (searchPredictedObjectsNearEntity(*msg_ptr)) {
    std::get<PredictedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(this->get_logger(), "Reacted %s", node_name.c_str());

    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void ReactionAnalyzerNode::detectedObjectsOutputCallback(
  const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  const auto is_spawned = spawn_cmd_time_;
  if (!std::holds_alternative<DetectedObjectsBuffer>(variant)) {
    DetectedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();
  if (
    !is_spawned || msg_ptr->objects.empty() ||
    std::get<DetectedObjectsBuffer>(variant).has_value()) {
    return;
  }

  // transform objects
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "map", msg_ptr->header.frame_id, this->now(), rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to look up transform from " << msg_ptr->header.frame_id << " to map");
    return;
  }

  DetectedObjects output_objs;
  output_objs = *msg_ptr;
  for (auto & obj : output_objs.objects) {
    geometry_msgs::msg::PoseStamped output_stamped, input_stamped;
    input_stamped.pose = obj.kinematics.pose_with_covariance.pose;
    tf2::doTransform(input_stamped, output_stamped, transform_stamped);
    obj.kinematics.pose_with_covariance.pose = output_stamped.pose;
  }
  if (searchDetectedObjectsNearEntity(output_objs)) {
    std::get<DetectedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(this->get_logger(), "Reacted %s", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void ReactionAnalyzerNode::trackedObjectsOutputCallback(
  const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  const auto is_spawned = spawn_cmd_time_;
  if (!std::holds_alternative<TrackedObjectsBuffer>(variant)) {
    TrackedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();
  if (
    !is_spawned || msg_ptr->objects.empty() ||
    std::get<TrackedObjectsBuffer>(variant).has_value()) {
    return;
  }

  if (searchTrackedObjectsNearEntity(*msg_ptr)) {
    std::get<TrackedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(this->get_logger(), "Reacted %s", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void ReactionAnalyzerNode::operationModeCallback(OperationModeState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  operation_mode_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::routeStateCallback(RouteState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_route_state_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::vehiclePoseCallback(Odometry::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::initializationStateCallback(
  LocalizationInitializationState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  initialization_state_ptr_ = std::move(msg_ptr);
}

ReactionAnalyzerNode::ReactionAnalyzerNode(rclcpp::NodeOptions options)
: Node("reaction_analyzer_node", options.automatically_declare_parameters_from_overrides(true))
{
  using std::placeholders::_1;

  node_params_.running_mode = get_parameter("running_mode").as_string();

  // set running mode
  if (node_params_.running_mode == "planning_control") {
    node_running_mode_ = RunningMode::PlanningControl;
  } else if (node_params_.running_mode == "perception_planning") {
    node_running_mode_ = RunningMode::PerceptionPlanning;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid running mode. Node couldn't be initialized. Failed.");
    return;
  }

  node_params_.timer_period = get_parameter("timer_period").as_double();
  node_params_.test_iteration = get_parameter("test_iteration").as_int();
  node_params_.published_time_expire_duration =
    get_parameter("published_time_expire_duration").as_double();
  node_params_.output_file_path = get_parameter("output_file_path").as_string();
  node_params_.object_search_radius_offset =
    get_parameter("object_search_radius_offset").as_double();
  node_params_.spawn_time_after_init = get_parameter("spawn_time_after_init").as_double();
  node_params_.spawn_distance_threshold = get_parameter("spawn_distance_threshold").as_double();
  node_params_.spawned_pointcloud_sampling_distance =
    get_parameter("spawned_pointcloud_sampling_distance").as_double();
  node_params_.dummy_perception_publisher_period =
    get_parameter("dummy_perception_publisher_period").as_double();
  node_params_.debug_control_commands =
    get_parameter("first_brake_params.debug_control_commands").as_bool();
  node_params_.control_cmd_buffer_time_interval =
    get_parameter("first_brake_params.control_cmd_buffer_time_interval").as_double();
  node_params_.min_number_descending_order_control_cmd =
    get_parameter("first_brake_params.min_number_descending_order_control_cmd").as_int();
  node_params_.min_jerk_for_brake_cmd =
    get_parameter("first_brake_params.min_jerk_for_brake_cmd").as_double();

  // Position parameters
  node_params_.initial_pose.x = get_parameter("initialization_pose.x").as_double();
  node_params_.initial_pose.y = get_parameter("initialization_pose.y").as_double();
  node_params_.initial_pose.z = get_parameter("initialization_pose.z").as_double();
  node_params_.initial_pose.roll = get_parameter("initialization_pose.roll").as_double();
  node_params_.initial_pose.pitch = get_parameter("initialization_pose.pitch").as_double();
  node_params_.initial_pose.yaw = get_parameter("initialization_pose.yaw").as_double();

  node_params_.goal_pose.x = get_parameter("goal_pose.x").as_double();
  node_params_.goal_pose.y = get_parameter("goal_pose.y").as_double();
  node_params_.goal_pose.z = get_parameter("goal_pose.z").as_double();
  node_params_.goal_pose.roll = get_parameter("goal_pose.roll").as_double();
  node_params_.goal_pose.pitch = get_parameter("goal_pose.pitch").as_double();
  node_params_.goal_pose.yaw = get_parameter("goal_pose.yaw").as_double();

  node_params_.entity_params.x = get_parameter("entity_params.x").as_double();
  node_params_.entity_params.y = get_parameter("entity_params.y").as_double();
  node_params_.entity_params.z = get_parameter("entity_params.z").as_double();
  node_params_.entity_params.roll = get_parameter("entity_params.roll").as_double();
  node_params_.entity_params.pitch = get_parameter("entity_params.pitch").as_double();
  node_params_.entity_params.yaw = get_parameter("entity_params.yaw").as_double();
  node_params_.entity_params.x_l = get_parameter("entity_params.x_dimension").as_double();
  node_params_.entity_params.y_l = get_parameter("entity_params.y_dimension").as_double();
  node_params_.entity_params.z_l = get_parameter("entity_params.z_dimension").as_double();

  // initialize the reaction chain
  if (!loadChainModules()) {
    RCLCPP_ERROR(
      get_logger(), "Modules in chain are invalid. Node couldn't be initialized. Failed.");
    return;
  }

  initAnalyzerVariables();

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, std::bind(&ReactionAnalyzerNode::vehiclePoseCallback, this, _1),
    createSubscriptionOptions());
  sub_localization_init_state_ = create_subscription<LocalizationInitializationState>(
    "input/localization_initialization_state", rclcpp::QoS(1).transient_local(),
    std::bind(&ReactionAnalyzerNode::initializationStateCallback, this, _1),
    createSubscriptionOptions());
  sub_route_state_ = create_subscription<RouteState>(
    "input/routing_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::routeStateCallback, this, _1), createSubscriptionOptions());
  sub_operation_mode_ = create_subscription<OperationModeState>(
    "input/operation_mode_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::operationModeCallback, this, _1), createSubscriptionOptions());

  pub_goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("output/goal", rclcpp::QoS(1));

  if (node_running_mode_ == RunningMode::PlanningControl) {
    pub_initial_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "output/initialpose", rclcpp::QoS(1));
    pub_pointcloud_ = create_publisher<PointCloud2>("output/pointcloud", rclcpp::SensorDataQoS());
    pub_predicted_objects_ = create_publisher<PredictedObjects>("output/objects", rclcpp::QoS(1));

    client_change_to_autonomous_ =
      create_client<ChangeOperationMode>("service/change_to_autonomous");

    // init dummy perception publisher
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(node_params_.dummy_perception_publisher_period));
    dummy_perception_timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns,
      std::bind(&ReactionAnalyzerNode::dummyPerceptionPublisher, this));

  } else if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    // Create topic publishers
    topic_publisher_ptr_ =
      std::make_shared<topic_publisher::TopicPublisher>(this, spawn_object_cmd_, spawn_cmd_time_);
  }

  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(node_params_.timer_period));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ReactionAnalyzerNode::onTimer, this));
}

void ReactionAnalyzerNode::onTimer()
{
  mutex_.lock();
  const auto current_odometry_ptr = odometry_;
  const auto initialization_state_ptr = initialization_state_ptr_;
  const auto route_state_ptr = current_route_state_ptr_;
  const auto operation_mode_ptr = operation_mode_ptr_;
  const auto message_buffers = message_buffers_;
  const auto spawn_cmd_time = spawn_cmd_time_;
  const auto published_time_vector_map = published_time_vector_map_;
  mutex_.unlock();

  // Init the test environment

  if (!test_environment_init_time_) {
    initEgoForTest(initialization_state_ptr, route_state_ptr, operation_mode_ptr);
    return;
  }

  spawnObstacle(current_odometry_ptr->pose.pose.position);

  if (!spawn_cmd_time) return;

  if (!allReacted(message_buffers)) return;

  if (!is_output_printed_) {
    calculateResults(message_buffers, published_time_vector_map, spawn_cmd_time.value());
  } else {
    reset();
  }
}

void ReactionAnalyzerNode::dummyPerceptionPublisher()
{
  if (!spawn_object_cmd_) {
    // do not spawn it, send empty pointcloud
    pcl::PointCloud<pcl::PointXYZ> pcl_empty;
    PointCloud2 empty_pointcloud;
    PredictedObjects empty_predicted_objects;
    pcl::toROSMsg(pcl_empty, empty_pointcloud);

    const auto current_time = this->now();
    empty_pointcloud.header.frame_id = "base_link";
    empty_pointcloud.header.stamp = current_time;

    empty_predicted_objects.header.frame_id = "map";
    empty_predicted_objects.header.stamp = current_time;

    pub_pointcloud_->publish(empty_pointcloud);
    pub_predicted_objects_->publish(empty_predicted_objects);
  } else {
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped{};
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link", "map", this->now(), rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to look up transform from map to base_link");
      return;
    }

    // transform by using eigen matrix
    PointCloud2 transformed_points{};
    const Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(affine_matrix, *entity_pointcloud_ptr_, transformed_points);
    const auto current_time = this->now();

    transformed_points.header.frame_id = "base_link";
    transformed_points.header.stamp = current_time;

    predicted_objects_ptr_->header.frame_id = "map";
    predicted_objects_ptr_->header.stamp = current_time;

    pub_pointcloud_->publish(transformed_points);
    pub_predicted_objects_->publish(*predicted_objects_ptr_);
    if (!is_object_spawned_message_published_) {
      mutex_.lock();
      spawn_cmd_time_ = this->now();
      mutex_.unlock();
      is_object_spawned_message_published_ = true;
    }
  }
}

void ReactionAnalyzerNode::spawnObstacle(const geometry_msgs::msg::Point & ego_pose)
{
  if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    rclcpp::Duration time_diff = this->now() - test_environment_init_time_.value();
    if (time_diff > rclcpp::Duration::from_seconds(node_params_.spawn_time_after_init)) {
      if (!spawn_object_cmd_) {
        spawn_object_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "Spawn command is sent.");
      }
    }
  } else {
    if (
      tier4_autoware_utils::calcDistance3d(ego_pose, entity_pose_.position) <
      node_params_.spawn_distance_threshold) {
      if (!spawn_object_cmd_) {
        spawn_object_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "Spawn command is sent.");
      }
    }
  }
}

bool ReactionAnalyzerNode::allReacted(
  const std::unordered_map<std::string, BufferVariant> & message_buffers)
{
  bool all_reacted = true;
  for (const auto & [key, variant] : message_buffers) {
    if (auto * control_message = std::get_if<ControlCommandBuffer>(&variant)) {
      if (!control_message->second) {
        all_reacted = false;
      }
    } else if (auto * planning_message = std::get_if<TrajectoryBuffer>(&variant)) {
      if (!planning_message->has_value()) {
        all_reacted = false;
      }
    } else if (auto * pointcloud_message = std::get_if<PointCloud2Buffer>(&variant)) {
      if (!pointcloud_message->has_value()) {
        all_reacted = false;
      }
    } else if (auto * predicted_objects_message = std::get_if<PredictedObjectsBuffer>(&variant)) {
      if (!predicted_objects_message->has_value()) {
        all_reacted = false;
      }
    } else if (auto * detected_objects_message = std::get_if<DetectedObjectsBuffer>(&variant)) {
      if (!detected_objects_message->has_value()) {
        all_reacted = false;
      }
    } else if (auto * tracked_objects_message = std::get_if<TrackedObjectsBuffer>(&variant)) {
      if (!tracked_objects_message->has_value()) {
        all_reacted = false;
      }
    }
  }
  return all_reacted;
}

std::optional<size_t> findConjugatePublishedTimeIdx(
  const std::vector<PublishedTime> & published_time_vector, const rclcpp::Time & time)
{
  auto it = std::find_if(
    published_time_vector.begin(), published_time_vector.end(),
    [&time](const PublishedTime & timeInVector) { return timeInVector.header.stamp == time; });

  if (it != published_time_vector.end()) {
    return std::optional<int>(
      std::distance(published_time_vector.begin(), it));  // Return the index of the found time
  } else {
    return std::nullopt;
  }
}

void ReactionAnalyzerNode::calculateResults(
  const std::unordered_map<std::string, BufferVariant> & message_buffers,
  const std::unordered_map<std::string, std::vector<PublishedTime>> & published_time_vector_map,
  const rclcpp::Time & spawn_cmd_time)
{
  auto createDurationMs = [](const rclcpp::Time & start_time, const rclcpp::Time & end_time) {
    return static_cast<double>((end_time - start_time).nanoseconds()) / 1e6;
  };
  std::vector<std::pair<std::string, rclcpp::Time>> reaction_times;
  for (const auto & [key, variant] : message_buffers) {
    rclcpp::Time reaction_time;

    if (auto * control_message = std::get_if<ControlCommandBuffer>(&variant)) {
      if (control_message->second) {
        auto it = published_time_vector_map.find(key);
        if (it != published_time_vector_map.end()) {
          const auto & published_time_vec = it->second;
          const auto idx =
            findConjugatePublishedTimeIdx(published_time_vec, control_message->second->stamp);
          if (idx) {
            reaction_time = rclcpp::Time(published_time_vec.at(idx.value()).published_stamp);
          } else {
            RCLCPP_ERROR(
              this->get_logger(), "Published time for %s node is not found", key.c_str());

            reaction_time = control_message->second->stamp;
          }
        } else {
          // It might do not have a published time debug topic
          reaction_time = control_message->second->stamp;
        }
      }
    } else if (auto * planning_message = std::get_if<TrajectoryBuffer>(&variant)) {
      if (planning_message->has_value()) {
        auto it = published_time_vector_map.find(key);
        if (it != published_time_vector_map.end()) {
          const auto & published_time_vec = it->second;
          const auto idx = findConjugatePublishedTimeIdx(
            published_time_vec, planning_message->value().header.stamp);
          if (idx) {
            reaction_time = rclcpp::Time(published_time_vec.at(idx.value()).published_stamp);
          } else {
            RCLCPP_ERROR(
              this->get_logger(), "Published time for %s node is not found", key.c_str());

            reaction_time = planning_message->value().header.stamp;
          }
        } else {
          reaction_time = planning_message->value().header.stamp;
        }
      }
    } else if (auto * pointcloud_message = std::get_if<PointCloud2Buffer>(&variant)) {
      if (pointcloud_message->has_value()) {
        auto it = published_time_vector_map.find(key);
        if (it != published_time_vector_map.end()) {
          const auto & published_time_vec = it->second;
          const auto idx = findConjugatePublishedTimeIdx(
            published_time_vec, pointcloud_message->value().header.stamp);
          if (idx) {
            reaction_time = rclcpp::Time(published_time_vec.at(idx.value()).published_stamp);
          } else {
            RCLCPP_ERROR(
              this->get_logger(), "Published time for %s node is not found", key.c_str());

            reaction_time = pointcloud_message->value().header.stamp;
          }
        } else {
          reaction_time = pointcloud_message->value().header.stamp;
        }
      }
    } else if (auto * predicted_objects_message = std::get_if<PredictedObjectsBuffer>(&variant)) {
      if (predicted_objects_message->has_value()) {
        auto it = published_time_vector_map.find(key);
        if (it != published_time_vector_map.end()) {
          const auto & published_time_vec = it->second;
          const auto idx = findConjugatePublishedTimeIdx(
            published_time_vec, predicted_objects_message->value().header.stamp);
          if (idx) {
            reaction_time = rclcpp::Time(published_time_vec.at(idx.value()).published_stamp);
          } else {
            RCLCPP_ERROR(
              this->get_logger(), "Published time for %s node is not found", key.c_str());

            reaction_time = predicted_objects_message->value().header.stamp;
          }
        } else {
          reaction_time = predicted_objects_message->value().header.stamp;
        }
      }
    } else if (auto * detected_objects_message = std::get_if<DetectedObjectsBuffer>(&variant)) {
      if (detected_objects_message->has_value()) {
        auto it = published_time_vector_map.find(key);
        if (it != published_time_vector_map.end()) {
          const auto & published_time_vec = it->second;
          const auto idx = findConjugatePublishedTimeIdx(
            published_time_vec, detected_objects_message->value().header.stamp);
          if (idx) {
            reaction_time = rclcpp::Time(published_time_vec.at(idx.value()).published_stamp);
          } else {
            RCLCPP_ERROR(
              this->get_logger(), "Published time for %s node is not found", key.c_str());

            reaction_time = detected_objects_message->value().header.stamp;
          }
        } else {
          reaction_time = detected_objects_message->value().header.stamp;
        }
      }
    } else if (auto * tracked_objects_message = std::get_if<TrackedObjectsBuffer>(&variant)) {
      if (tracked_objects_message->has_value()) {
        auto it = published_time_vector_map.find(key);
        if (it != published_time_vector_map.end()) {
          const auto & published_time_vec = it->second;
          const auto idx = findConjugatePublishedTimeIdx(
            published_time_vec, tracked_objects_message->value().header.stamp);
          if (idx) {
            reaction_time = rclcpp::Time(published_time_vec.at(idx.value()).published_stamp);
          } else {
            RCLCPP_ERROR(
              this->get_logger(), "Published time for %s node is not found", key.c_str());

            reaction_time = tracked_objects_message->value().header.stamp;
          }
        } else {
          reaction_time = tracked_objects_message->value().header.stamp;
        }
      }
    }

    const auto duration = createDurationMs(spawn_cmd_time, reaction_time);

    RCLCPP_INFO(
      this->get_logger(), "Spawn time to %s node reaction: %lf ms", key.c_str(), duration);
    test_results_[key].emplace_back(duration);
  }
  test_iteration_count_++;
  is_output_printed_ = true;
}

bool ReactionAnalyzerNode::loadChainModules()
{
  auto split = [](const std::string & str, const char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delim)) {
      elems.push_back(item);
    }
    return elems;
  };

  auto stringToMessageType = [](const std::string & input) {
    if (input == "autoware_auto_control_msgs::msg::AckermannControlCommand") {
      return SubscriberMessageType::AckermannControlCommand;
    } else if (input == "autoware_auto_planning_msgs::msg::Trajectory") {
      return SubscriberMessageType::Trajectory;
    } else if (input == "sensor_msgs::msg::PointCloud2") {
      return SubscriberMessageType::PointCloud2;
    } else if (input == "autoware_auto_perception_msgs::msg::PredictedObjects") {
      return SubscriberMessageType::PredictedObjects;
    } else if (input == "autoware_auto_perception_msgs::msg::DetectedObjects") {
      return SubscriberMessageType::DetectedObjects;
    } else if (input == "autoware_auto_perception_msgs::msg::TrackedObjects") {
      return SubscriberMessageType::TrackedObjects;
    } else {
      return SubscriberMessageType::Unknown;
    }
  };

  // get the topic addresses and message types of the modules in chain
  const auto param_key = std::string("reaction_chain");
  const auto module_names = this->list_parameters({param_key}, 3).prefixes;
  ChainModules chain_modules;
  for (const auto & module_name : module_names) {
    const auto splitted_name = split(module_name, '.');
    TopicConfig tmp;
    tmp.node_name = splitted_name.back();
    tmp.topic_address = this->get_parameter(module_name + ".topic_name").as_string();
    tmp.time_debug_topic_address =
      this->get_parameter_or(module_name + ".time_debug_topic_name", std::string(""));
    tmp.message_type =
      stringToMessageType(this->get_parameter(module_name + ".message_type").as_string());
    if (tmp.message_type != SubscriberMessageType::Unknown) {
      chain_modules.emplace_back(tmp);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Unknown message type for module name: %s, skipping..",
        tmp.node_name.c_str());
    }
  }
  return (initSubscribers(chain_modules));
}

void ReactionAnalyzerNode::initAnalyzerVariables()
{
  tf2::Quaternion entity_q_orientation;
  entity_q_orientation.setRPY(
    tier4_autoware_utils::deg2rad(node_params_.entity_params.roll),
    tier4_autoware_utils::deg2rad(node_params_.entity_params.pitch),
    tier4_autoware_utils::deg2rad(node_params_.entity_params.yaw));
  entity_pose_.position.set__x(node_params_.entity_params.x);
  entity_pose_.position.set__y(node_params_.entity_params.y);
  entity_pose_.position.set__z(node_params_.entity_params.z);
  entity_pose_.orientation.set__x(entity_q_orientation.x());
  entity_pose_.orientation.set__y(entity_q_orientation.y());
  entity_pose_.orientation.set__z(entity_q_orientation.z());
  entity_pose_.orientation.set__w(entity_q_orientation.w());

  // find minimum radius of sphere that encloses the entity

  entity_search_radius_ =
    std::sqrt(
      std::pow(node_params_.entity_params.x_l, 2) + std::pow(node_params_.entity_params.y_l, 2) +
      std::pow(node_params_.entity_params.z_l, 2)) /
      2.0 +
    node_params_.object_search_radius_offset;

  tf2::Quaternion goal_pose_q_orientation;
  goal_pose_q_orientation.setRPY(
    tier4_autoware_utils::deg2rad(node_params_.goal_pose.roll),
    tier4_autoware_utils::deg2rad(node_params_.goal_pose.pitch),
    tier4_autoware_utils::deg2rad(node_params_.goal_pose.yaw));

  goal_pose_.pose.position.set__x(node_params_.goal_pose.x);
  goal_pose_.pose.position.set__y(node_params_.goal_pose.y);
  goal_pose_.pose.position.set__z(node_params_.goal_pose.z);
  goal_pose_.pose.orientation.set__x(goal_pose_q_orientation.x());
  goal_pose_.pose.orientation.set__y(goal_pose_q_orientation.y());
  goal_pose_.pose.orientation.set__z(goal_pose_q_orientation.z());
  goal_pose_.pose.orientation.set__w(goal_pose_q_orientation.w());

  if (node_running_mode_ == RunningMode::PlanningControl) {
    tf2::Quaternion initial_pose_q_orientation;
    initial_pose_q_orientation.setRPY(
      tier4_autoware_utils::deg2rad(node_params_.initial_pose.roll),
      tier4_autoware_utils::deg2rad(node_params_.initial_pose.pitch),
      tier4_autoware_utils::deg2rad(node_params_.initial_pose.yaw));

    init_pose_.pose.pose.position.set__x(node_params_.initial_pose.x);
    init_pose_.pose.pose.position.set__y(node_params_.initial_pose.y);
    init_pose_.pose.pose.position.set__z(node_params_.initial_pose.z);
    init_pose_.pose.pose.orientation.set__x(initial_pose_q_orientation.x());
    init_pose_.pose.pose.orientation.set__y(initial_pose_q_orientation.y());
    init_pose_.pose.pose.orientation.set__z(initial_pose_q_orientation.z());
    init_pose_.pose.pose.orientation.set__w(initial_pose_q_orientation.w());

    initPointcloud();
    initPredictedObjects();
  }
}

void ReactionAnalyzerNode::initPointcloud()
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  // prepare transform matrix
  tf2::Quaternion entity_q_orientation;
  entity_q_orientation.setX(entity_pose_.orientation.x);
  entity_q_orientation.setY(entity_pose_.orientation.y);
  entity_q_orientation.setZ(entity_pose_.orientation.z);
  entity_q_orientation.setW(entity_pose_.orientation.w);

  tf2::Transform tf(entity_q_orientation);
  const auto origin =
    tf2::Vector3(entity_pose_.position.x, entity_pose_.position.y, entity_pose_.position.z);
  tf.setOrigin(origin);

  const double it_x =
    node_params_.entity_params.x_l / node_params_.spawned_pointcloud_sampling_distance;
  const double it_y =
    node_params_.entity_params.y_l / node_params_.spawned_pointcloud_sampling_distance;
  const double it_z =
    node_params_.entity_params.z_l / node_params_.spawned_pointcloud_sampling_distance;

  // Sample the box and rotate
  for (int i = 0; i <= it_z; ++i) {
    for (int j = 0; j <= it_y; ++j) {
      for (int k = 0; k <= it_x; ++k) {
        const double p_x = -node_params_.entity_params.x_l / 2 +
                           k * node_params_.spawned_pointcloud_sampling_distance;
        const double p_y = -node_params_.entity_params.y_l / 2 +
                           j * node_params_.spawned_pointcloud_sampling_distance;
        const double p_z = -node_params_.entity_params.z_l / 2 +
                           i * node_params_.spawned_pointcloud_sampling_distance;
        const auto tmp = tf2::Vector3(p_x, p_y, p_z);
        tf2::Vector3 data_out = tf * tmp;
        point_cloud.emplace_back(pcl::PointXYZ(data_out.x(), data_out.y(), data_out.z()));
      }
    }
  }
  entity_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::toROSMsg(point_cloud, *entity_pointcloud_ptr_);
}

bool ReactionAnalyzerNode::initSubscribers(const reaction_analyzer::ChainModules & modules)
{
  if (modules.empty()) {
    RCLCPP_ERROR(get_logger(), "No module to initialize, failed.");
    return false;
  }
  for (const auto & module : modules) {
    if (!module.time_debug_topic_address.empty()) {
      auto callback = [this, module](const PublishedTime::ConstSharedPtr & msg) {
        this->publishedTimeOutputCallback(module.node_name, msg);
      };
      auto subscriber = this->create_subscription<PublishedTime>(
        module.time_debug_topic_address, rclcpp::QoS(1), callback, createSubscriptionOptions());
      subscribers_.push_back(subscriber);
      published_time_vector_map_[module.node_name] = std::vector<PublishedTime>();
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Time debug topic is not provided for module name: %s, skipping..",
        module.node_name.c_str());
    }
    switch (module.message_type) {
      case SubscriberMessageType::AckermannControlCommand: {
        auto callback = [this, module](const AckermannControlCommand::ConstSharedPtr & msg) {
          this->controlCommandOutputCallback(module.node_name, msg);
        };
        auto subscriber = this->create_subscription<AckermannControlCommand>(
          module.topic_address, rclcpp::QoS(1), callback, createSubscriptionOptions());
        subscribers_.push_back(subscriber);
        break;
      }
      case SubscriberMessageType::Trajectory: {
        auto callback = [this, module](const Trajectory::ConstSharedPtr & msg) {
          this->trajectoryOutputCallback(module.node_name, msg);
        };
        auto subscriber = this->create_subscription<Trajectory>(
          module.topic_address, rclcpp::QoS(1), callback, createSubscriptionOptions());
        subscribers_.push_back(subscriber);
        break;
      }
      case SubscriberMessageType::PointCloud2: {
        auto callback = [this, module](const PointCloud2::ConstSharedPtr & msg) {
          this->pointcloud2OutputCallback(module.node_name, msg);
        };
        auto subscriber = this->create_subscription<PointCloud2>(
          module.topic_address, rclcpp::SensorDataQoS(), callback, createSubscriptionOptions());
        subscribers_.push_back(subscriber);
        break;
      }
      case SubscriberMessageType::PredictedObjects: {
        auto callback = [this, module](const PredictedObjects::ConstSharedPtr & msg) {
          this->predictedObjectsOutputCallback(module.node_name, msg);
        };
        auto subscriber = this->create_subscription<PredictedObjects>(
          module.topic_address, rclcpp::QoS(1), callback, createSubscriptionOptions());
        subscribers_.push_back(subscriber);
        break;
      }
      case SubscriberMessageType::DetectedObjects: {
        auto callback = [this, module](const DetectedObjects::ConstSharedPtr & msg) {
          this->detectedObjectsOutputCallback(module.node_name, msg);
        };
        auto subscriber = this->create_subscription<DetectedObjects>(
          module.topic_address, rclcpp::QoS(1), callback, createSubscriptionOptions());
        subscribers_.push_back(subscriber);
        break;
      }
      case SubscriberMessageType::TrackedObjects: {
        auto callback = [this, module](const TrackedObjects::ConstSharedPtr & msg) {
          this->trackedObjectsOutputCallback(module.node_name, msg);
        };
        auto subscriber = this->create_subscription<TrackedObjects>(
          module.topic_address, rclcpp::QoS(1), callback, createSubscriptionOptions());
        subscribers_.push_back(subscriber);
        break;
      }
      case SubscriberMessageType::Unknown:
        RCLCPP_WARN(
          this->get_logger(), "Unknown message type for module name: %s, skipping..",
          module.node_name.c_str());
        break;
      default:
        RCLCPP_WARN(
          this->get_logger(), "Unknown message type for module name: %s, skipping..",
          module.node_name.c_str());
        break;
    }
  }
  if (subscribers_.empty()) {
    RCLCPP_ERROR(
      get_logger(), "Subscribers for modules are empty. Node couldn't be initialized. Failed");
    return false;
  }
  return true;
}

void ReactionAnalyzerNode::initPredictedObjects()
{
  auto generateUUIDMsg = [](const std::string & input) {
    static auto generate_uuid = boost::uuids::name_generator(boost::uuids::random_generator()());
    const auto uuid = generate_uuid(input);

    unique_identifier_msgs::msg::UUID uuid_msg;
    std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());
    return uuid_msg;
  };

  PredictedObject obj;

  geometry_msgs::msg::Vector3 dimension;
  dimension.set__x(node_params_.entity_params.x_l);
  dimension.set__y(node_params_.entity_params.y_l);
  dimension.set__z(node_params_.entity_params.z_l);
  obj.shape.set__dimensions(dimension);

  obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.existence_probability = 1.0;
  obj.kinematics.initial_pose_with_covariance.pose = entity_pose_;
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  obj.classification.emplace_back(classification);
  obj.set__object_id(generateUUIDMsg("test_obstacle"));
  PredictedObjects pred_objects;
  pred_objects.objects.emplace_back(obj);
  predicted_objects_ptr_ = std::make_shared<PredictedObjects>(pred_objects);
}

void ReactionAnalyzerNode::setControlCommandToBuffer(
  std::vector<AckermannControlCommand> & buffer, const AckermannControlCommand & cmd)
{
  const auto last_cmd_time = cmd.stamp;
  if (!buffer.empty()) {
    for (auto itr = buffer.begin(); itr != buffer.end();) {
      const auto expired = (rclcpp::Time(last_cmd_time) - rclcpp::Time(itr->stamp)).seconds() >
                           node_params_.control_cmd_buffer_time_interval;

      if (expired) {
        itr = buffer.erase(itr);
        continue;
      }

      itr++;
    }
  }
  buffer.emplace_back(cmd);
}

void ReactionAnalyzerNode::pushPublishedTime(
  std::vector<PublishedTime> & published_time_vec, const PublishedTime & published_time)
{
  published_time_vec.emplace_back(published_time);
  if (published_time_vec.size() > 1) {
    for (auto itr = published_time_vec.begin(); itr != published_time_vec.end();) {
      const auto expired =
        (rclcpp::Time(published_time.header.stamp) - rclcpp::Time(itr->header.stamp)).seconds() >
        node_params_.published_time_expire_duration;

      if (expired) {
        itr = published_time_vec.erase(itr);
        continue;
      }

      itr++;
    }
  }
}

std::optional<size_t> ReactionAnalyzerNode::findFirstBrakeIdx(
  const std::vector<AckermannControlCommand> & cmd_array,
  const std::optional<rclcpp::Time> & spawn_cmd_time)
{
  if (
    cmd_array.size() < static_cast<size_t>(node_params_.min_number_descending_order_control_cmd) ||
    !spawn_cmd_time)
    return {};

  // wait for enough data after spawn_cmd_time
  if (
    rclcpp::Time(
      cmd_array.at(cmd_array.size() - node_params_.min_number_descending_order_control_cmd).stamp) <
    spawn_cmd_time)
    return {};

  for (size_t i = 0;
       i < cmd_array.size() - node_params_.min_number_descending_order_control_cmd + 1; ++i) {
    size_t decreased_cmd_counter = 1;  // because # of the decreased cmd = iteration + 1
    for (size_t j = i; j < cmd_array.size() - 1; ++j) {
      const auto & cmd_first = cmd_array.at(j).longitudinal;
      const auto & cmd_second = cmd_array.at(j + 1).longitudinal;
      constexpr double jerk_time_epsilon = 0.001;
      const auto jerk =
        abs(cmd_second.acceleration - cmd_first.acceleration) /
        std::max(
          (rclcpp::Time(cmd_second.stamp) - rclcpp::Time(cmd_first.stamp)).seconds(),
          jerk_time_epsilon);

      if (
        (cmd_second.acceleration < cmd_first.acceleration) &&
        (jerk > node_params_.min_jerk_for_brake_cmd)) {
        decreased_cmd_counter++;
      } else {
        break;
      }
    }
    if (
      decreased_cmd_counter <
      static_cast<size_t>(node_params_.min_number_descending_order_control_cmd))
      continue;
    if (node_params_.debug_control_commands) {
      std::stringstream ss;

      // debug print to show the first brake command in the all control commands
      for (size_t k = 0; k < cmd_array.size(); ++k) {
        if (k == i + 1) {
          ss << "First Brake(" << cmd_array.at(k).longitudinal.acceleration << ") ";
        } else {
          ss << cmd_array.at(k).longitudinal.acceleration << " ";
        }
      }

      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
    return i + 1;
  }
  return {};
}

void ReactionAnalyzerNode::initEgoForTest(
  const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
  const RouteState::ConstSharedPtr & route_state_ptr,
  const OperationModeState::ConstSharedPtr & operation_mode_ptr)
{
  const auto current_time = this->now();

  // Initialize the vehicle
  constexpr double initialize_call_period = 1.0;  // sec

  if (
    !last_test_environment_init_request_time_ ||
    (current_time - last_test_environment_init_request_time_.value()).seconds() >=
      initialize_call_period) {
    last_test_environment_init_request_time_ = current_time;

    if (
      initialization_state_ptr &&
      (initialization_state_ptr->state != LocalizationInitializationState::INITIALIZED ||
       !is_vehicle_initialized_)) {
      if (initialization_state_ptr->state == LocalizationInitializationState::INITIALIZED) {
        is_vehicle_initialized_ = true;
      }
      if (node_running_mode_ == RunningMode::PlanningControl) {
        // publish initial pose
        init_pose_.header.stamp = current_time;
        init_pose_.header.frame_id = "map";
        pub_initial_pose_->publish(init_pose_);
      }
      return;
    }

    if (route_state_ptr && (route_state_ptr->state != RouteState::SET || !is_route_set_)) {
      if (route_state_ptr->state == RouteState::SET) {
        is_route_set_ = true;
      }
      // publish goal pose
      goal_pose_.header.stamp = current_time;
      goal_pose_.header.frame_id = "map";
      pub_goal_pose_->publish(goal_pose_);
      return;
    }

    if (node_running_mode_ == RunningMode::PlanningControl) {
      // change to autonomous
      if (operation_mode_ptr && operation_mode_ptr->mode != OperationModeState::AUTONOMOUS) {
        callOperationModeServiceWithoutResponse();
        return;
      }
    }
    const bool is_ready =
      (is_vehicle_initialized_ && is_route_set_ &&
       (operation_mode_ptr->mode == OperationModeState::AUTONOMOUS ||
        node_running_mode_ == RunningMode::PerceptionPlanning));
    if (is_ready) {
      test_environment_init_time_ = this->now();
    }
  }
}

rclcpp::SubscriptionOptions ReactionAnalyzerNode::createSubscriptionOptions()
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}

void ReactionAnalyzerNode::callOperationModeServiceWithoutResponse()
{
  auto req = std::make_shared<ChangeOperationMode::Request>();

  RCLCPP_INFO(this->get_logger(), "client request");

  if (!client_change_to_autonomous_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "client is unavailable");
    return;
  }

  client_change_to_autonomous_->async_send_request(
    req, [this](typename rclcpp::Client<ChangeOperationMode>::SharedFuture result) {
      RCLCPP_INFO(
        this->get_logger(), "Status: %d, %s", result.get()->status.code,
        result.get()->status.message.c_str());
    });
}

bool ReactionAnalyzerNode::searchPointcloudNearEntity(
  const pcl::PointCloud<pcl::PointXYZ> & pcl_pointcloud)
{
  bool isAnyPointWithinRadius = std::any_of(
    pcl_pointcloud.points.begin(), pcl_pointcloud.points.end(), [this](const auto & point) {
      return tier4_autoware_utils::calcDistance3d(entity_pose_.position, point) <=
             entity_search_radius_;
    });

  if (isAnyPointWithinRadius) {
    return true;
  }
  return false;
}

bool ReactionAnalyzerNode::searchPredictedObjectsNearEntity(
  const PredictedObjects & predicted_objects)
{
  bool isAnyObjectWithinRadius = std::any_of(
    predicted_objects.objects.begin(), predicted_objects.objects.end(),
    [this](const PredictedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               entity_pose_.position,
               object.kinematics.initial_pose_with_covariance.pose.position) <=
             entity_search_radius_;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

bool ReactionAnalyzerNode::searchDetectedObjectsNearEntity(const DetectedObjects & detected_objects)
{
  bool isAnyObjectWithinRadius = std::any_of(
    detected_objects.objects.begin(), detected_objects.objects.end(),
    [this](const DetectedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               entity_pose_.position, object.kinematics.pose_with_covariance.pose.position) <=
             entity_search_radius_;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

bool ReactionAnalyzerNode::searchTrackedObjectsNearEntity(const TrackedObjects & tracked_objects)
{
  bool isAnyObjectWithinRadius = std::any_of(
    tracked_objects.objects.begin(), tracked_objects.objects.end(),
    [this](const TrackedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               entity_pose_.position, object.kinematics.pose_with_covariance.pose.position) <=
             entity_search_radius_;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

void ReactionAnalyzerNode::reset()
{
  if (test_iteration_count_ >= node_params_.test_iteration) {
    writeResultsToFile();
    RCLCPP_INFO(get_logger(), "%zu Tests are finished. Node shutting down.", test_iteration_count_);
    rclcpp::shutdown();
    return;
  }
  is_vehicle_initialized_ = false;
  is_route_set_ = false;
  test_environment_init_time_ = std::nullopt;
  last_test_environment_init_request_time_ = std::nullopt;
  spawn_object_cmd_ = false;
  is_output_printed_ = false;
  is_object_spawned_message_published_ = false;
  if (topic_publisher_ptr_) {
    topic_publisher_ptr_->reset();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  message_buffers_.clear();
  spawn_cmd_time_ = std::nullopt;
  for (auto & [key, value] : published_time_vector_map_) {
    value.clear();
  }
  RCLCPP_INFO(this->get_logger(), "Test - %zu is done, resetting..", test_iteration_count_);
}

void ReactionAnalyzerNode::writeResultsToFile()
{
  // sort the results w.r.t the median value

  const auto sort_by_median =
    [this]() -> std::vector<std::tuple<std::string, std::vector<double>, double>> {
    std::vector<std::tuple<std::string, std::vector<double>, double>> sorted_data;

    for (const auto & pair : test_results_) {
      auto vec = pair.second;

      // Calculate the median
      std::sort(vec.begin(), vec.end());
      double median = 0.0;
      size_t size = vec.size();
      if (size % 2 == 0) {
        median = (vec[size / 2 - 1] + vec[size / 2]) / 2.0;
      } else {
        median = vec[size / 2];
      }

      sorted_data.emplace_back(pair.first, pair.second, median);
    }

    // Sort based on the computed median
    std::sort(sorted_data.begin(), sorted_data.end(), [](const auto & a, const auto & b) {
      return std::get<2>(a) < std::get<2>(b);  // Change to > for descending order
    });

    return sorted_data;
  };

  const auto sorted_data_by_median = sort_by_median();

  // create csv file
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << node_params_.output_file_path;
  if (!node_params_.output_file_path.empty() && node_params_.output_file_path.back() != '/') {
    ss << "/";  // Ensure the path ends with a slash
  }
  if (node_running_mode_ == RunningMode::PlanningControl) {
    ss << "planning_control-";
  } else {
    ss << "perception_planning-";
  }

  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  ss << "-reaction-results.csv";

  // parse the results
  std::ofstream file(ss.str());
  // Check if the file was opened successfully
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << ss.str() << std::endl;
    return;
  }

  bool is_header_written = false;

  for (const auto & data : sorted_data_by_median) {
    // Unpack the tuple
    const auto & node = std::get<0>(data);
    const auto & durations = std::get<1>(data);

    if (!is_header_written) {
      file << "Node,";
      const size_t num_durations = durations.size();
      for (size_t i = 0; i < num_durations; ++i) {
        file << "Test - " << i + 1 << ",";
      }
      file << "Min,Max,Mean,Median,StdDev\n";
      is_header_written = true;
    }

    // parse test results
    file << node << ",";
    for (double duration : durations) {
      file << duration << ",";
    }

    // calculate stats
    const double min = *std::min_element(durations.begin(), durations.end());
    const double max = *std::max_element(durations.begin(), durations.end());

    std::vector<double> sorted_data = durations;
    std::sort(sorted_data.begin(), sorted_data.end());
    const double median =
      sorted_data.size() % 2 == 0
        ? (sorted_data[sorted_data.size() / 2 - 1] + sorted_data[sorted_data.size() / 2]) / 2
        : sorted_data[sorted_data.size() / 2];

    const double mean =
      std::accumulate(sorted_data.begin(), sorted_data.end(), 0.0) / sorted_data.size();
    const double sq_sum = std::inner_product(
      sorted_data.begin(), sorted_data.end(), sorted_data.begin(), 0.0, std::plus<>(),
      [&](double a, double b) { return (a - mean) * (b - mean); });
    double std_dev = std::sqrt(sq_sum / sorted_data.size());

    // parse stats
    file << min << "," << max << "," << mean << "," << median << "," << std_dev << "\n";
  }
  file.close();
  RCLCPP_INFO(this->get_logger(), "Results written to: %s", ss.str().c_str());
}

}  // namespace reaction_analyzer

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>

RCLCPP_COMPONENTS_REGISTER_NODE(reaction_analyzer::ReactionAnalyzerNode)
