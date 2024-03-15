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

#include "subscriber.hpp"

#include <algorithm>
#include <memory>
#include <utility>

namespace reaction_analyzer::subscriber
{

SubscriberBase::SubscriberBase(
  rclcpp::Node * node, Odometry::ConstSharedPtr & odometry,
  const geometry_msgs::msg::Pose entity_pose, std::atomic<bool> & spawn_object_cmd)
: node_(node), odometry_(odometry), entity_pose_(entity_pose), spawn_object_cmd_(spawn_object_cmd)
{
  // init tf
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // init reaction parameters and chain configuration
  init_reaction_chains_and_params();
  init_subscribers();
}

void SubscriberBase::init_reaction_chains_and_params()
{
  auto stringToMessageType = [](const std::string & input) {
    if (input == "autoware_auto_control_msgs/msg/AckermannControlCommand") {
      return SubscriberMessageType::ACKERMANN_CONTROL_COMMAND;
    } else if (input == "autoware_auto_planning_msgs/msg/Trajectory") {
      return SubscriberMessageType::TRAJECTORY;
    } else if (input == "sensor_msgs/msg/PointCloud2") {
      return SubscriberMessageType::POINTCLOUD2;
    } else if (input == "autoware_auto_perception_msgs/msg/PredictedObjects") {
      return SubscriberMessageType::PREDICTED_OBJECTS;
    } else if (input == "autoware_auto_perception_msgs/msg/DetectedObjects") {
      return SubscriberMessageType::DETECTED_OBJECTS;
    } else if (input == "autoware_auto_perception_msgs/msg/TrackedObjects") {
      return SubscriberMessageType::TRACKED_OBJECTS;
    } else {
      return SubscriberMessageType::UNKNOWN;
    }
  };

  auto stringToReactionType = [](const std::string & input) {
    if (input == "first_brake_params") {
      return ReactionType::FIRST_BRAKE;
    } else if (input == "search_zero_vel_params") {
      return ReactionType::SEARCH_ZERO_VEL;
    } else if (input == "search_entity_params") {
      return ReactionType::SEARCH_ENTITY;
    } else {
      return ReactionType::UNKNOWN;
    }
  };

  // Init Chains: get the topic addresses and message types of the modules in chain
  {
    const auto param_key = std::string("subscriber.reaction_chain");
    const auto module_names = node_->list_parameters({param_key}, 3).prefixes;
    for (const auto & module_name : module_names) {
      const auto splitted_name = split(module_name, '.');
      TopicConfig tmp;
      tmp.node_name = splitted_name.back();
      tmp.topic_address = node_->get_parameter(module_name + ".topic_name").as_string();
      tmp.time_debug_topic_address =
        node_->get_parameter_or(module_name + ".time_debug_topic_name", std::string(""));
      tmp.message_type =
        stringToMessageType(node_->get_parameter(module_name + ".message_type").as_string());
      if (tmp.message_type != SubscriberMessageType::UNKNOWN) {
        chain_modules_.emplace_back(tmp);
      } else {
        RCLCPP_WARN(
          node_->get_logger(), "Unknown message type for module name: %s, skipping..",
          tmp.node_name.c_str());
      }
    }
  }

  // Init Params: get the parameters for the reaction functions
  {
    const auto param_key = std::string("subscriber.reaction_params");
    const auto module_names = node_->list_parameters({param_key}, 3).prefixes;
    for (const auto & module_name : module_names) {
      const auto splitted_name = split(module_name, '.');
      const auto type = stringToReactionType(splitted_name.back());
      switch (type) {
        case ReactionType::FIRST_BRAKE: {
          reaction_params_.first_brake_params.debug_control_commands =
            node_->get_parameter(module_name + ".debug_control_commands").as_bool();
          reaction_params_.first_brake_params.control_cmd_buffer_time_interval =
            node_->get_parameter(module_name + ".control_cmd_buffer_time_interval").as_double();
          reaction_params_.first_brake_params.min_number_descending_order_control_cmd =
            node_->get_parameter(module_name + ".min_number_descending_order_control_cmd").as_int();
          reaction_params_.first_brake_params.min_jerk_for_brake_cmd =
            node_->get_parameter(module_name + ".min_jerk_for_brake_cmd").as_double();
          RCLCPP_INFO_ONCE(
            node_->get_logger(),
            "First brake parameters are set: debug_control_commands %s, "
            "control_cmd_buffer_time_interval %f, min_number_descending_order_control_cmd %zu, "
            "min_jerk_for_brake_cmd %f",
            reaction_params_.first_brake_params.debug_control_commands ? "true" : "false",
            reaction_params_.first_brake_params.control_cmd_buffer_time_interval,
            reaction_params_.first_brake_params.min_number_descending_order_control_cmd,
            reaction_params_.first_brake_params.min_jerk_for_brake_cmd);
          break;
        }
        case ReactionType::SEARCH_ZERO_VEL: {
          reaction_params_.search_zero_vel_params.max_looking_distance =
            node_->get_parameter(module_name + ".max_looking_distance").as_double();
          RCLCPP_INFO_ONCE(
            node_->get_logger(), "Search Zero Vel parameters are set: max_looking_distance %f",
            reaction_params_.search_zero_vel_params.max_looking_distance);
          break;
        }
        case ReactionType::SEARCH_ENTITY: {
          reaction_params_.search_entity_params.search_radius =
            node_->get_parameter(module_name + ".search_radius").as_double();
          RCLCPP_INFO_ONCE(
            node_->get_logger(), "Search Entity parameters are set: search_radius %f",
            reaction_params_.search_entity_params.search_radius);
          break;
        }
        default:
          RCLCPP_WARN(
            node_->get_logger(), "Unknown reaction type for module name: %s, skipping..",
            splitted_name.back().c_str());
          break;
      }
    }
  }
}

bool SubscriberBase::init_subscribers()
{
  if (chain_modules_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No module to initialize subscribers, failed.");
    return false;
  }

  for (const auto & module : chain_modules_) {
    switch (module.message_type) {
      case SubscriberMessageType::ACKERMANN_CONTROL_COMMAND: {
        SubscriberVariables<AckermannControlCommand> subscriber_variable;

        if (!module.time_debug_topic_address.empty()) {
          // If not empty, user should define a time debug topic
          // NOTE: Because message_filters package does not support the messages without headers, we
          // can not use the synchronizer. After we reacted, we are going to use the cache
          // of the both PublishedTime and AckermannControlCommand subscribers to find the messages
          // which have same header time.

          std::function<void(const AckermannControlCommand::ConstSharedPtr &)> callback =
            [this, module](const AckermannControlCommand::ConstSharedPtr & ptr) {
              this->on_control_command(module.node_name, ptr);
            };
          subscriber_variable.sub1_ =
            std::make_unique<message_filters::Subscriber<AckermannControlCommand>>(
              node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
              create_subscription_options(node_));

          subscriber_variable.sub1_->registerCallback(std::bind(callback, std::placeholders::_1));

          subscriber_variable.sub2_ = std::make_unique<message_filters::Subscriber<PublishedTime>>(
            node_, module.time_debug_topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));
          constexpr int cache_size = 5;
          subscriber_variable.cache_ = std::make_unique<message_filters::Cache<PublishedTime>>(
            *subscriber_variable.sub2_, cache_size);

        } else {
          std::function<void(const AckermannControlCommand::ConstSharedPtr &)> callback =
            [this, module](const AckermannControlCommand::ConstSharedPtr & ptr) {
              this->on_control_command(module.node_name, ptr);
            };

          subscriber_variable.sub1_ =
            std::make_unique<message_filters::Subscriber<AckermannControlCommand>>(
              node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
              create_subscription_options(node_));

          subscriber_variable.sub1_->registerCallback(std::bind(callback, std::placeholders::_1));
          RCLCPP_WARN(
            node_->get_logger(),
            "PublishedTime will not be used for node name: %s. Header timestamp will be used for "
            "calculations",
            module.node_name.c_str());
        }
        // set the variable to map with the topic address
        subscriber_variables_map_[module.node_name] = std::move(subscriber_variable);
        break;
      }
      case SubscriberMessageType::TRAJECTORY: {
        SubscriberVariables<Trajectory> subscriber_variable;

        if (!module.time_debug_topic_address.empty()) {
          std::function<void(
            const Trajectory::ConstSharedPtr &, const PublishedTime::ConstSharedPtr &)>
            callback = [this, module](
                         const Trajectory::ConstSharedPtr & ptr,
                         const PublishedTime::ConstSharedPtr & published_time_ptr) {
              this->on_trajectory(module.node_name, ptr, published_time_ptr);
            };

          subscriber_variable.sub1_ = std::make_unique<message_filters::Subscriber<Trajectory>>(
            node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));
          subscriber_variable.sub2_ = std::make_unique<message_filters::Subscriber<PublishedTime>>(
            node_, module.time_debug_topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.synchronizer_ = std::make_unique<
            message_filters::Synchronizer<SubscriberVariables<Trajectory>::ExactTimePolicy>>(
            SubscriberVariables<Trajectory>::ExactTimePolicy(10), *subscriber_variable.sub1_,
            *subscriber_variable.sub2_);

          subscriber_variable.synchronizer_->registerCallback(
            std::bind(callback, std::placeholders::_1, std::placeholders::_2));

        } else {
          std::function<void(const Trajectory::ConstSharedPtr &)> callback =
            [this, module](const Trajectory::ConstSharedPtr & msg) {
              this->on_trajectory(module.node_name, msg);
            };
          subscriber_variable.sub1_ = std::make_unique<message_filters::Subscriber<Trajectory>>(
            node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.sub1_->registerCallback(std::bind(callback, std::placeholders::_1));
          RCLCPP_WARN(
            node_->get_logger(),
            "PublishedTime will not be used for node name: %s. Header timestamp will be used for "
            "calculations",
            module.node_name.c_str());
        }
        subscriber_variables_map_[module.node_name] = std::move(subscriber_variable);
        break;
      }
      case SubscriberMessageType::POINTCLOUD2: {
        SubscriberVariables<PointCloud2> subscriber_variable;

        if (!module.time_debug_topic_address.empty()) {
          std::function<void(
            const PointCloud2::ConstSharedPtr &, const PublishedTime::ConstSharedPtr &)>
            callback = [this, module](
                         const PointCloud2::ConstSharedPtr & ptr,
                         const PublishedTime::ConstSharedPtr & published_time_ptr) {
              this->on_pointcloud(module.node_name, ptr, published_time_ptr);
            };

          subscriber_variable.sub1_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(
            node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));
          subscriber_variable.sub2_ = std::make_unique<message_filters::Subscriber<PublishedTime>>(
            node_, module.time_debug_topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.synchronizer_ = std::make_unique<
            message_filters::Synchronizer<SubscriberVariables<PointCloud2>::ExactTimePolicy>>(
            SubscriberVariables<PointCloud2>::ExactTimePolicy(10), *subscriber_variable.sub1_,
            *subscriber_variable.sub2_);

          subscriber_variable.synchronizer_->registerCallback(
            std::bind(callback, std::placeholders::_1, std::placeholders::_2));

        } else {
          std::function<void(const PointCloud2::ConstSharedPtr &)> callback =
            [this, module](const PointCloud2::ConstSharedPtr & msg) {
              this->on_pointcloud(module.node_name, msg);
            };
          subscriber_variable.sub1_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(
            node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.sub1_->registerCallback(std::bind(callback, std::placeholders::_1));
          RCLCPP_WARN(
            node_->get_logger(),
            "PublishedTime will not be used for node name: %s. Header timestamp will be used for "
            "calculations",
            module.node_name.c_str());
        }
        subscriber_variables_map_[module.node_name] = std::move(subscriber_variable);
        break;
      }
      case SubscriberMessageType::PREDICTED_OBJECTS: {
        SubscriberVariables<PredictedObjects> subscriber_variable;

        if (!module.time_debug_topic_address.empty()) {
          std::function<void(
            const PredictedObjects::ConstSharedPtr &, const PublishedTime::ConstSharedPtr &)>
            callback = [this, module](
                         const PredictedObjects::ConstSharedPtr & ptr,
                         const PublishedTime::ConstSharedPtr & published_time_ptr) {
              this->on_predicted_objects(module.node_name, ptr, published_time_ptr);
            };
          subscriber_variable.sub1_ =
            std::make_unique<message_filters::Subscriber<PredictedObjects>>(
              node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
              create_subscription_options(node_));
          subscriber_variable.sub2_ = std::make_unique<message_filters::Subscriber<PublishedTime>>(
            node_, module.time_debug_topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.synchronizer_ = std::make_unique<
            message_filters::Synchronizer<SubscriberVariables<PredictedObjects>::ExactTimePolicy>>(
            SubscriberVariables<PredictedObjects>::ExactTimePolicy(10), *subscriber_variable.sub1_,
            *subscriber_variable.sub2_);

          subscriber_variable.synchronizer_->registerCallback(
            std::bind(callback, std::placeholders::_1, std::placeholders::_2));

        } else {
          std::function<void(const PredictedObjects::ConstSharedPtr &)> callback =
            [this, module](const PredictedObjects::ConstSharedPtr & msg) {
              this->on_predicted_objects(module.node_name, msg);
            };
          subscriber_variable.sub1_ =
            std::make_unique<message_filters::Subscriber<PredictedObjects>>(
              node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
              create_subscription_options(node_));

          subscriber_variable.sub1_->registerCallback(std::bind(callback, std::placeholders::_1));
          RCLCPP_WARN(
            node_->get_logger(),
            "PublishedTime will not be used for node name: %s. Header timestamp will be used for "
            "calculations",
            module.node_name.c_str());
        }
        subscriber_variables_map_[module.node_name] = std::move(subscriber_variable);
        break;
      }
      case SubscriberMessageType::DETECTED_OBJECTS: {
        SubscriberVariables<DetectedObjects> subscriber_variable;

        if (!module.time_debug_topic_address.empty()) {
          std::function<void(
            const DetectedObjects::ConstSharedPtr &, const PublishedTime::ConstSharedPtr &)>
            callback = [this, module](
                         const DetectedObjects::ConstSharedPtr & ptr,
                         const PublishedTime::ConstSharedPtr & published_time_ptr) {
              this->on_detected_objects(module.node_name, ptr, published_time_ptr);
            };

          subscriber_variable.sub1_ =
            std::make_unique<message_filters::Subscriber<DetectedObjects>>(
              node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
              create_subscription_options(node_));
          subscriber_variable.sub2_ = std::make_unique<message_filters::Subscriber<PublishedTime>>(
            node_, module.time_debug_topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.synchronizer_ = std::make_unique<
            message_filters::Synchronizer<SubscriberVariables<DetectedObjects>::ExactTimePolicy>>(
            SubscriberVariables<DetectedObjects>::ExactTimePolicy(10), *subscriber_variable.sub1_,
            *subscriber_variable.sub2_);

          subscriber_variable.synchronizer_->registerCallback(
            std::bind(callback, std::placeholders::_1, std::placeholders::_2));

        } else {
          std::function<void(const DetectedObjects::ConstSharedPtr &)> callback =
            [this, module](const DetectedObjects::ConstSharedPtr & msg) {
              this->on_detected_objects(module.node_name, msg);
            };
          subscriber_variable.sub1_ =
            std::make_unique<message_filters::Subscriber<DetectedObjects>>(
              node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
              create_subscription_options(node_));

          subscriber_variable.sub1_->registerCallback(std::bind(callback, std::placeholders::_1));
          RCLCPP_WARN(
            node_->get_logger(),
            "PublishedTime will not be used for node name: %s. Header timestamp will be used for "
            "calculations",
            module.node_name.c_str());
        }
        subscriber_variables_map_[module.node_name] = std::move(subscriber_variable);

        break;
      }
      case SubscriberMessageType::TRACKED_OBJECTS: {
        SubscriberVariables<TrackedObjects> subscriber_variable;
        if (!module.time_debug_topic_address.empty()) {
          std::function<void(
            const TrackedObjects::ConstSharedPtr &, const PublishedTime::ConstSharedPtr &)>
            callback = [this, module](
                         const TrackedObjects::ConstSharedPtr & ptr,
                         const PublishedTime::ConstSharedPtr & published_time_ptr) {
              this->on_tracked_objects(module.node_name, ptr, published_time_ptr);
            };

          subscriber_variable.sub1_ = std::make_unique<message_filters::Subscriber<TrackedObjects>>(
            node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));
          subscriber_variable.sub2_ = std::make_unique<message_filters::Subscriber<PublishedTime>>(
            node_, module.time_debug_topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.synchronizer_ = std::make_unique<
            message_filters::Synchronizer<SubscriberVariables<TrackedObjects>::ExactTimePolicy>>(
            SubscriberVariables<TrackedObjects>::ExactTimePolicy(10), *subscriber_variable.sub1_,
            *subscriber_variable.sub2_);

          subscriber_variable.synchronizer_->registerCallback(
            std::bind(callback, std::placeholders::_1, std::placeholders::_2));

        } else {
          std::function<void(const TrackedObjects::ConstSharedPtr &)> callback =
            [this, module](const TrackedObjects::ConstSharedPtr & msg) {
              this->on_tracked_objects(module.node_name, msg);
            };
          subscriber_variable.sub1_ = std::make_unique<message_filters::Subscriber<TrackedObjects>>(
            node_, module.topic_address, rclcpp::QoS(1).get_rmw_qos_profile(),
            create_subscription_options(node_));

          subscriber_variable.sub1_->registerCallback(std::bind(callback, std::placeholders::_1));
          RCLCPP_WARN(
            node_->get_logger(),
            "PublishedTime will not be used for node name: %s. Header timestamp will be used for "
            "calculations",
            module.node_name.c_str());
        }
        subscriber_variables_map_[module.node_name] = std::move(subscriber_variable);
        break;
      }
      case SubscriberMessageType::UNKNOWN:
        RCLCPP_WARN(
          node_->get_logger(), "Unknown message type for module name: %s, skipping..",
          module.node_name.c_str());
        break;
      default:
        RCLCPP_WARN(
          node_->get_logger(), "Unknown message type for module name: %s, skipping..",
          module.node_name.c_str());
        break;
    }
  }
  return true;
}

std::optional<std::unordered_map<std::string, BufferVariant>> SubscriberBase::getMessageBuffersMap()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (message_buffers_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No message buffers are initialized.");
    return std::nullopt;
  }

  // Check all reacted or not
  bool all_reacted = true;
  for (const auto & [key, variant] : message_buffers_) {
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
  if (!all_reacted) {
    return std::nullopt;
  }
  return message_buffers_;
}

void SubscriberBase::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  message_buffers_.clear();
}

// Callbacks

void SubscriberBase::on_control_command(
  const std::string & node_name, const AckermannControlCommand::ConstSharedPtr & msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & variant = message_buffers_[node_name];
  if (!std::holds_alternative<ControlCommandBuffer>(variant)) {
    ControlCommandBuffer buffer(std::vector<AckermannControlCommand>{*msg_ptr}, std::nullopt);
    variant = buffer;
  }
  auto & cmd_buffer = std::get<ControlCommandBuffer>(variant);
  if (cmd_buffer.second) {
    // reacted
    return;
  }
  set_control_command_to_buffer(cmd_buffer.first, *msg_ptr);
  if (!spawn_object_cmd_) return;

  const auto brake_idx = find_first_brake_idx(cmd_buffer.first);
  if (brake_idx) {
    const auto brake_cmd = cmd_buffer.first.at(brake_idx.value());

    // TODO(brkay54): update here if message_filters package add the support for the messages which
    // does not have header
    const auto & subscriber_variant =
      std::get<SubscriberVariables<AckermannControlCommand>>(subscriber_variables_map_[node_name]);

    // check if the cache was initialized or not, if there is, use it to set the published time
    if (subscriber_variant.cache_) {
      // use cache to get the published time
      const auto published_time_vec =
        subscriber_variant.cache_->getSurroundingInterval(brake_cmd.stamp, node_->now());
      if (!published_time_vec.empty()) {
        for (const auto & published_time : published_time_vec) {
          if (published_time->header.stamp == brake_cmd.stamp) {
            cmd_buffer.second = brake_cmd;
            cmd_buffer.second->stamp = published_time->published_stamp;
            RCLCPP_INFO(node_->get_logger(), "%s reacted with published time", node_name.c_str());
            return;
          }
        }
        RCLCPP_ERROR(
          node_->get_logger(), "Published time couldn't found for the node: %s", node_name.c_str());

      } else {
        RCLCPP_ERROR(
          node_->get_logger(), "Published time vector is empty for the node: %s",
          node_name.c_str());
      }
    } else {
      cmd_buffer.second = brake_cmd;
      RCLCPP_INFO(node_->get_logger(), "%s reacted without published time", node_name.c_str());
    }
  }
}

void SubscriberBase::on_trajectory(
  const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  const auto current_odometry_ptr = odometry_;
  if (!std::holds_alternative<TrajectoryBuffer>(variant)) {
    TrajectoryBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (
    !current_odometry_ptr || !spawn_object_cmd_ ||
    std::get<TrajectoryBuffer>(variant).has_value()) {
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
    RCLCPP_INFO(node_->get_logger(), "%s reacted without published time", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void SubscriberBase::on_trajectory(
  const std::string & node_name, const Trajectory::ConstSharedPtr & msg_ptr,
  const PublishedTime::ConstSharedPtr & published_time_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  const auto current_odometry_ptr = odometry_;
  if (!std::holds_alternative<TrajectoryBuffer>(variant)) {
    TrajectoryBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (
    !current_odometry_ptr || !spawn_object_cmd_ ||
    std::get<TrajectoryBuffer>(variant).has_value()) {
    return;
  }

  const auto nearest_seg_idx = motion_utils::findNearestSegmentIndex(
    msg_ptr->points, current_odometry_ptr->pose.pose.position);

  // find the target index which we will search for zero velocity
  auto tmp_target_idx = get_index_after_distance(
    *msg_ptr, nearest_seg_idx, reaction_params_.search_zero_vel_params.max_looking_distance);
  if (tmp_target_idx == msg_ptr->points.size() - 1) {
    tmp_target_idx = msg_ptr->points.size() - 2;  // Last trajectory points might be zero velocity
  }
  const auto target_idx = tmp_target_idx;
  const auto zero_vel_idx =
    motion_utils::searchZeroVelocityIndex(msg_ptr->points, nearest_seg_idx, target_idx);

  if (zero_vel_idx) {
    std::get<TrajectoryBuffer>(variant) = *msg_ptr;

    // set published time
    std::get<TrajectoryBuffer>(variant)->header.stamp = published_time_ptr->published_stamp;
    RCLCPP_INFO(node_->get_logger(), "%s reacted with published time", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void SubscriberBase::on_pointcloud(
  const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<PointCloud2Buffer>(variant)) {
    PointCloud2Buffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (!spawn_object_cmd_ || std::get<PointCloud2Buffer>(variant).has_value()) {
    return;
  }

  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      "map", msg_ptr->header.frame_id, node_->now(), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Failed to look up transform from " << msg_ptr->header.frame_id << " to map");
    return;
  }

  // transform by using eigen matrix
  PointCloud2 transformed_points{};
  const Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(affine_matrix, *msg_ptr, transformed_points);

  pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
  pcl::fromROSMsg(transformed_points, pcl_pointcloud);

  if (search_pointcloud_near_entity(pcl_pointcloud)) {
    std::get<PointCloud2Buffer>(variant) = *msg_ptr;
    RCLCPP_INFO(node_->get_logger(), "%s reacted without published time", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}
void SubscriberBase::on_pointcloud(
  const std::string & node_name, const PointCloud2::ConstSharedPtr & msg_ptr,
  const PublishedTime::ConstSharedPtr & published_time_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<PointCloud2Buffer>(variant)) {
    PointCloud2Buffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (!spawn_object_cmd_ || std::get<PointCloud2Buffer>(variant).has_value()) {
    return;
  }

  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      "map", msg_ptr->header.frame_id, node_->now(), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Failed to look up transform from " << msg_ptr->header.frame_id << " to map");
    return;
  }

  // transform by using eigen matrix
  PointCloud2 transformed_points{};
  const Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(affine_matrix, *msg_ptr, transformed_points);

  pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
  pcl::fromROSMsg(transformed_points, pcl_pointcloud);

  if (search_pointcloud_near_entity(pcl_pointcloud)) {
    std::get<PointCloud2Buffer>(variant) = *msg_ptr;
    // set published time
    std::get<PointCloud2Buffer>(variant)->header.stamp = published_time_ptr->published_stamp;
    RCLCPP_INFO(node_->get_logger(), "%s reacted with published time", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}
void SubscriberBase::on_predicted_objects(
  const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<PredictedObjectsBuffer>(variant)) {
    PredictedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (
    !spawn_object_cmd_ || msg_ptr->objects.empty() ||
    std::get<PredictedObjectsBuffer>(variant).has_value()) {
    return;
  }

  if (search_predicted_objects_near_entity(*msg_ptr)) {
    std::get<PredictedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(node_->get_logger(), "%s reacted without published time", node_name.c_str());

    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void SubscriberBase::on_predicted_objects(
  const std::string & node_name, const PredictedObjects::ConstSharedPtr & msg_ptr,
  const PublishedTime::ConstSharedPtr & published_time_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<PredictedObjectsBuffer>(variant)) {
    PredictedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();

  if (
    !spawn_object_cmd_ || msg_ptr->objects.empty() ||
    std::get<PredictedObjectsBuffer>(variant).has_value()) {
    return;
  }

  if (search_predicted_objects_near_entity(*msg_ptr)) {
    std::get<PredictedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(node_->get_logger(), "%s reacted with published time", node_name.c_str());

    // set published time
    std::get<PredictedObjectsBuffer>(variant)->header.stamp = published_time_ptr->published_stamp;
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void SubscriberBase::on_detected_objects(
  const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<DetectedObjectsBuffer>(variant)) {
    DetectedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();
  if (
    !spawn_object_cmd_ || msg_ptr->objects.empty() ||
    std::get<DetectedObjectsBuffer>(variant).has_value()) {
    return;
  }

  // transform objects
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      "map", msg_ptr->header.frame_id, node_->now(), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Failed to look up transform from " << msg_ptr->header.frame_id << " to map");
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
  if (search_detected_objects_near_entity(output_objs)) {
    std::get<DetectedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(node_->get_logger(), "%s reacted without published time", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void SubscriberBase::on_detected_objects(
  const std::string & node_name, const DetectedObjects::ConstSharedPtr & msg_ptr,
  const PublishedTime::ConstSharedPtr & published_time_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<DetectedObjectsBuffer>(variant)) {
    DetectedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();
  if (
    !spawn_object_cmd_ || msg_ptr->objects.empty() ||
    std::get<DetectedObjectsBuffer>(variant).has_value()) {
    return;
  }

  // transform objects
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      "map", msg_ptr->header.frame_id, node_->now(), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Failed to look up transform from " << msg_ptr->header.frame_id << " to map");
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
  if (search_detected_objects_near_entity(output_objs)) {
    std::get<DetectedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(node_->get_logger(), "%s reacted with published time", node_name.c_str());

    // set published time
    std::get<DetectedObjectsBuffer>(variant)->header.stamp = published_time_ptr->published_stamp;
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void SubscriberBase::on_tracked_objects(
  const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<TrackedObjectsBuffer>(variant)) {
    TrackedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();
  if (
    !spawn_object_cmd_ || msg_ptr->objects.empty() ||
    std::get<TrackedObjectsBuffer>(variant).has_value()) {
    return;
  }

  if (search_tracked_objects_near_entity(*msg_ptr)) {
    std::get<TrackedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(node_->get_logger(), "Reacted %s", node_name.c_str());
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

void SubscriberBase::on_tracked_objects(
  const std::string & node_name, const TrackedObjects::ConstSharedPtr & msg_ptr,
  const PublishedTime::ConstSharedPtr & published_time_ptr)
{
  mutex_.lock();
  auto variant = message_buffers_[node_name];
  if (!std::holds_alternative<TrackedObjectsBuffer>(variant)) {
    TrackedObjectsBuffer buffer(std::nullopt);
    variant = buffer;
    message_buffers_[node_name] = variant;
  }
  mutex_.unlock();
  if (
    !spawn_object_cmd_ || msg_ptr->objects.empty() ||
    std::get<TrackedObjectsBuffer>(variant).has_value()) {
    return;
  }

  if (search_tracked_objects_near_entity(*msg_ptr)) {
    std::get<TrackedObjectsBuffer>(variant) = *msg_ptr;
    RCLCPP_INFO(node_->get_logger(), "%s reacted with published time", node_name.c_str());
    // set published time
    std::get<TrackedObjectsBuffer>(variant)->header.stamp = published_time_ptr->published_stamp;
    mutex_.lock();
    message_buffers_[node_name] = variant;
    mutex_.unlock();
  }
}

bool SubscriberBase::search_pointcloud_near_entity(
  const pcl::PointCloud<pcl::PointXYZ> & pcl_pointcloud)
{
  bool isAnyPointWithinRadius = std::any_of(
    pcl_pointcloud.points.begin(), pcl_pointcloud.points.end(), [this](const auto & point) {
      return tier4_autoware_utils::calcDistance3d(entity_pose_.position, point) <=
             reaction_params_.search_entity_params.search_radius;
    });

  if (isAnyPointWithinRadius) {
    return true;
  }
  return false;
}

bool SubscriberBase::search_predicted_objects_near_entity(
  const PredictedObjects & predicted_objects)
{
  bool isAnyObjectWithinRadius = std::any_of(
    predicted_objects.objects.begin(), predicted_objects.objects.end(),
    [this](const PredictedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               entity_pose_.position,
               object.kinematics.initial_pose_with_covariance.pose.position) <=
             reaction_params_.search_entity_params.search_radius;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

bool SubscriberBase::search_detected_objects_near_entity(const DetectedObjects & detected_objects)
{
  bool isAnyObjectWithinRadius = std::any_of(
    detected_objects.objects.begin(), detected_objects.objects.end(),
    [this](const DetectedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               entity_pose_.position, object.kinematics.pose_with_covariance.pose.position) <=
             reaction_params_.search_entity_params.search_radius;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

bool SubscriberBase::search_tracked_objects_near_entity(const TrackedObjects & tracked_objects)
{
  bool isAnyObjectWithinRadius = std::any_of(
    tracked_objects.objects.begin(), tracked_objects.objects.end(),
    [this](const TrackedObject & object) {
      return tier4_autoware_utils::calcDistance3d(
               entity_pose_.position, object.kinematics.pose_with_covariance.pose.position) <=
             reaction_params_.search_entity_params.search_radius;
    });

  if (isAnyObjectWithinRadius) {
    return true;
  }
  return false;
}

std::optional<size_t> SubscriberBase::find_first_brake_idx(
  const std::vector<AckermannControlCommand> & cmd_array)
{
  if (
    cmd_array.size() <
    reaction_params_.first_brake_params.min_number_descending_order_control_cmd) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "Control command buffer size is less than the minimum required size for first brake check");
    return {};
  }

  for (size_t i = 0;
       i < cmd_array.size() -
             reaction_params_.first_brake_params.min_number_descending_order_control_cmd + 1;
       ++i) {
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
        (jerk > reaction_params_.first_brake_params.min_jerk_for_brake_cmd)) {
        decreased_cmd_counter++;
      } else {
        break;
      }
    }
    if (
      decreased_cmd_counter <
      static_cast<size_t>(
        reaction_params_.first_brake_params.min_number_descending_order_control_cmd))
      continue;
    if (reaction_params_.first_brake_params.debug_control_commands) {
      std::stringstream ss;

      // debug print to show the first brake command in the all control commands
      for (size_t k = 0; k < cmd_array.size(); ++k) {
        if (k == i + 1) {
          ss << "First Brake(" << cmd_array.at(k).longitudinal.acceleration << ") ";
        } else {
          ss << cmd_array.at(k).longitudinal.acceleration << " ";
        }
      }

      RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
    }
    return i + 1;
  }
  return {};
}

void SubscriberBase::set_control_command_to_buffer(
  std::vector<AckermannControlCommand> & buffer, const AckermannControlCommand & cmd)
{
  const auto last_cmd_time = cmd.stamp;
  if (!buffer.empty()) {
    for (auto itr = buffer.begin(); itr != buffer.end();) {
      const auto expired = (rclcpp::Time(last_cmd_time) - rclcpp::Time(itr->stamp)).seconds() >
                           reaction_params_.first_brake_params.control_cmd_buffer_time_interval;

      if (expired) {
        itr = buffer.erase(itr);
        continue;
      }

      itr++;
    }
  }
  buffer.emplace_back(cmd);
}
}  // namespace reaction_analyzer::subscriber
