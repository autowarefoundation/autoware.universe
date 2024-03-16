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

#include <functional>
#include <memory>

namespace reaction_analyzer
{

void ReactionAnalyzerNode::on_operation_mode_state(OperationModeState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  operation_mode_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::on_route_state(RouteState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_route_state_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::on_vehicle_pose(Odometry::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_ptr_ = msg_ptr;
}

void ReactionAnalyzerNode::on_localization_initialization_state(
  LocalizationInitializationState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  initialization_state_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::on_ground_truth_pose(PoseStamped::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ground_truth_pose_ptr_ = std::move(msg_ptr);
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
  node_params_.output_file_path = get_parameter("output_file_path").as_string();

  node_params_.spawn_time_after_init = get_parameter("spawn_time_after_init").as_double();
  node_params_.spawn_distance_threshold = get_parameter("spawn_distance_threshold").as_double();
  node_params_.spawned_pointcloud_sampling_distance =
    get_parameter("spawned_pointcloud_sampling_distance").as_double();
  node_params_.dummy_perception_publisher_period =
    get_parameter("dummy_perception_publisher_period").as_double();

  // Position parameters
  node_params_.initial_pose.x = get_parameter("poses.initialization_pose.x").as_double();
  node_params_.initial_pose.y = get_parameter("poses.initialization_pose.y").as_double();
  node_params_.initial_pose.z = get_parameter("poses.initialization_pose.z").as_double();
  node_params_.initial_pose.roll = get_parameter("poses.initialization_pose.roll").as_double();
  node_params_.initial_pose.pitch = get_parameter("poses.initialization_pose.pitch").as_double();
  node_params_.initial_pose.yaw = get_parameter("poses.initialization_pose.yaw").as_double();

  node_params_.goal_pose.x = get_parameter("poses.goal_pose.x").as_double();
  node_params_.goal_pose.y = get_parameter("poses.goal_pose.y").as_double();
  node_params_.goal_pose.z = get_parameter("poses.goal_pose.z").as_double();
  node_params_.goal_pose.roll = get_parameter("poses.goal_pose.roll").as_double();
  node_params_.goal_pose.pitch = get_parameter("poses.goal_pose.pitch").as_double();
  node_params_.goal_pose.yaw = get_parameter("poses.goal_pose.yaw").as_double();

  node_params_.entity_params.x = get_parameter("poses.entity_params.x").as_double();
  node_params_.entity_params.y = get_parameter("poses.entity_params.y").as_double();
  node_params_.entity_params.z = get_parameter("poses.entity_params.z").as_double();
  node_params_.entity_params.roll = get_parameter("poses.entity_params.roll").as_double();
  node_params_.entity_params.pitch = get_parameter("poses.entity_params.pitch").as_double();
  node_params_.entity_params.yaw = get_parameter("poses.entity_params.yaw").as_double();
  node_params_.entity_params.x_l = get_parameter("poses.entity_params.x_dimension").as_double();
  node_params_.entity_params.y_l = get_parameter("poses.entity_params.y_dimension").as_double();
  node_params_.entity_params.z_l = get_parameter("poses.entity_params.z_dimension").as_double();

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, std::bind(&ReactionAnalyzerNode::on_vehicle_pose, this, _1),
    create_subscription_options(this));
  sub_localization_init_state_ = create_subscription<LocalizationInitializationState>(
    "input/localization_initialization_state", rclcpp::QoS(1).transient_local(),
    std::bind(&ReactionAnalyzerNode::on_localization_initialization_state, this, _1),
    create_subscription_options(this));
  sub_route_state_ = create_subscription<RouteState>(
    "input/routing_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::on_route_state, this, _1), create_subscription_options(this));
  sub_operation_mode_ = create_subscription<OperationModeState>(
    "input/operation_mode_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::on_operation_mode_state, this, _1),
    create_subscription_options(this));

  pub_goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("output/goal", rclcpp::QoS(1));

  init_analyzer_variables();

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
      std::bind(&ReactionAnalyzerNode::dummy_perception_publisher, this));

  } else if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    // Create topic publishers
    topic_publisher_ptr_ =
      std::make_unique<topic_publisher::TopicPublisher>(this, spawn_object_cmd_, spawn_cmd_time_);

    // Subscribe to the ground truth position
    sub_ground_truth_pose_ = create_subscription<PoseStamped>(
      "input/ground_truth_pose", rclcpp::QoS{1},
      std::bind(&ReactionAnalyzerNode::on_ground_truth_pose, this, _1),
      create_subscription_options(this));
  }

  // Load the subscriber to listen the topics for reactions
  odometry_ptr_ =
    std::make_shared<Odometry>();  // initialize the odometry before init the subscriber
  subscriber_ptr_ = std::make_unique<subscriber::SubscriberBase>(
    this, odometry_ptr_, entity_pose_, spawn_object_cmd_);

  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(node_params_.timer_period));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ReactionAnalyzerNode::on_timer, this));
}

void ReactionAnalyzerNode::on_timer()
{
  mutex_.lock();
  const auto current_odometry_ptr = odometry_ptr_;
  const auto initialization_state_ptr = initialization_state_ptr_;
  const auto route_state_ptr = current_route_state_ptr_;
  const auto operation_mode_ptr = operation_mode_ptr_;
  const auto ground_truth_pose_ptr = ground_truth_pose_ptr_;
  const auto spawn_cmd_time = spawn_cmd_time_;
  mutex_.unlock();

  // Init the test environment
  if (!test_environment_init_time_) {
    init_ego(
      initialization_state_ptr, route_state_ptr, operation_mode_ptr, ground_truth_pose_ptr,
      current_odometry_ptr);
    return;
  }

  // Spawn the obstacle if the conditions are met
  spawn_obstacle(current_odometry_ptr->pose.pose.position);

  // If the spawn_cmd_time is not set by pointcloud publishers, don't do anything
  if (!spawn_cmd_time) return;

  // Get the reacted messages, if all of the modules reacted
  const auto message_buffers = subscriber_ptr_->getMessageBuffersMap();

  if (message_buffers) {
    // if reacted, calculate the results
    calculate_results(message_buffers.value(), spawn_cmd_time.value());
    // reset the variables if the iteration is not finished, otherwise write the results
    reset();
  }
}

void ReactionAnalyzerNode::dummy_perception_publisher()
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
        "base_link", "map", this->now(), rclcpp::Duration::from_seconds(0.1));
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

void ReactionAnalyzerNode::spawn_obstacle(const geometry_msgs::msg::Point & ego_pose)
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

void ReactionAnalyzerNode::calculate_results(
  const std::unordered_map<std::string, subscriber::BufferVariant> & message_buffers,
  const rclcpp::Time & spawn_cmd_time)
{
  auto createDurationMs = [](const rclcpp::Time & start_time, const rclcpp::Time & end_time) {
    return static_cast<double>((end_time - start_time).nanoseconds()) / 1e6;
  };
  std::vector<std::pair<std::string, rclcpp::Time>> reaction_times;
  for (const auto & [key, variant] : message_buffers) {
    rclcpp::Time reaction_time;

    if (auto * control_message = std::get_if<subscriber::ControlCommandBuffer>(&variant)) {
      if (control_message->second) {
        reaction_time = control_message->second->stamp;
      }
    } else if (auto * planning_message = std::get_if<subscriber::TrajectoryBuffer>(&variant)) {
      if (planning_message->has_value()) {
        reaction_time = planning_message->value().header.stamp;
      }
    } else if (auto * pointcloud_message = std::get_if<subscriber::PointCloud2Buffer>(&variant)) {
      if (pointcloud_message->has_value()) {
        reaction_time = pointcloud_message->value().header.stamp;
      }
    } else if (
      auto * predicted_objects_message =
        std::get_if<subscriber::PredictedObjectsBuffer>(&variant)) {
      if (predicted_objects_message->has_value()) {
        reaction_time = predicted_objects_message->value().header.stamp;
      }
    } else if (
      auto * detected_objects_message = std::get_if<subscriber::DetectedObjectsBuffer>(&variant)) {
      if (detected_objects_message->has_value()) {
        reaction_time = detected_objects_message->value().header.stamp;
      }
    } else if (
      auto * tracked_objects_message = std::get_if<subscriber::TrackedObjectsBuffer>(&variant)) {
      if (tracked_objects_message->has_value()) {
        reaction_time = tracked_objects_message->value().header.stamp;
      }
    }
    const auto duration = createDurationMs(spawn_cmd_time, reaction_time);

    RCLCPP_INFO(
      this->get_logger(), "Spawn time to %s node reaction: %lf ms", key.c_str(), duration);
    test_results_[key].emplace_back(duration);
  }
  test_iteration_count_++;
}

void ReactionAnalyzerNode::init_analyzer_variables()
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

    init_pointcloud();
    init_predicted_objects();
  }
}

void ReactionAnalyzerNode::init_pointcloud()
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

void ReactionAnalyzerNode::init_predicted_objects()
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

  autoware_auto_perception_msgs::msg::PredictedPath path;
  path.confidence = 1.0;
  path.path.emplace_back(entity_pose_);
  obj.kinematics.predicted_paths.emplace_back(path);

  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  obj.classification.emplace_back(classification);
  obj.set__object_id(generateUUIDMsg("test_obstacle"));

  PredictedObjects pred_objects;
  pred_objects.objects.emplace_back(obj);
  predicted_objects_ptr_ = std::make_shared<PredictedObjects>(pred_objects);
}

void ReactionAnalyzerNode::init_ego(
  const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
  const RouteState::ConstSharedPtr & route_state_ptr,
  const OperationModeState::ConstSharedPtr & operation_mode_ptr,
  const PoseStamped::ConstSharedPtr & ground_truth_pose_ptr,
  const Odometry::ConstSharedPtr & odometry_ptr)
{
  const auto current_time = this->now();

  // Initialize the test environment
  constexpr double initialize_call_period = 1.0;  // sec

  if (
    !last_test_environment_init_request_time_ ||
    (current_time - last_test_environment_init_request_time_.value()).seconds() >=
      initialize_call_period) {
    last_test_environment_init_request_time_ = current_time;

    // Pose initialization of the ego
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

    // Wait until odometry_ptr is initialized
    if (!odometry_ptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Odometry is not received. Waiting for odometry...");
      return;
    }

    // Check is position initialized accurately, if node is running PerceptionPlanning mode
    if (node_running_mode_ == RunningMode::PerceptionPlanning) {
      if (!ground_truth_pose_ptr) {
        RCLCPP_WARN(
          get_logger(), "Ground truth pose is not received. Waiting for Ground truth pose...");
        return;
      } else {
        constexpr double deviation_threshold = 0.1;
        const auto deviation = tier4_autoware_utils::calcPoseDeviation(
          ground_truth_pose_ptr->pose, odometry_ptr->pose.pose);
        if (
          deviation.longitudinal > deviation_threshold || deviation.lateral > deviation_threshold ||
          deviation.yaw > deviation_threshold) {
          RCLCPP_ERROR(
            get_logger(),
            "Deviation between ground truth position and ego position is high. Node is shutting "
            "down. Longitudinal deviation: %f, Lateral deviation: %f, Yaw deviation: %f",
            deviation.longitudinal, deviation.lateral, deviation.yaw);
          rclcpp::shutdown();
        }
      }
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

    // if node is running PlanningControl mode, change ego to Autonomous mode.
    if (node_running_mode_ == RunningMode::PlanningControl) {
      // change to autonomous
      if (operation_mode_ptr && operation_mode_ptr->mode != OperationModeState::AUTONOMOUS) {
        call_operation_mode_service_without_response();
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

void ReactionAnalyzerNode::call_operation_mode_service_without_response()
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

void ReactionAnalyzerNode::reset()
{
  if (test_iteration_count_ >= node_params_.test_iteration) {
    write_results();
    RCLCPP_INFO(get_logger(), "%zu Tests are finished. Node shutting down.", test_iteration_count_);
    rclcpp::shutdown();
    return;
  }
  is_vehicle_initialized_ = false;
  is_route_set_ = false;
  test_environment_init_time_ = std::nullopt;
  last_test_environment_init_request_time_ = std::nullopt;
  spawn_object_cmd_ = false;
  is_object_spawned_message_published_ = false;
  if (topic_publisher_ptr_) {
    topic_publisher_ptr_->reset();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  spawn_cmd_time_ = std::nullopt;
  subscriber_ptr_->reset();
  RCLCPP_INFO(this->get_logger(), "Test - %zu is done, resetting..", test_iteration_count_);
}

void ReactionAnalyzerNode::write_results()
{
  // sort the results w.r.t the median value
  const auto sorted_data_by_median = sort_results_by_median(test_results_);

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
