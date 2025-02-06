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

#include "autoware/behavior_velocity_planner/node.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/ros/wait_for_param.hpp>
#include <autoware/universe_utils/transform/transforms.hpp>
#include <autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/Route.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <functional>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace
{

autoware_planning_msgs::msg::Path to_path(
  const tier4_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_planning_msgs::msg::Path path;
  for (const auto & path_point : path_with_id.points) {
    path.points.push_back(path_point.point);
  }
  return path;
}
}  // namespace

BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_velocity_planner_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  planner_data_(*this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<tier4_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id", 1, std::bind(&BehaviorVelocityPlannerNode::onTrigger, this, _1));

  srv_load_plugin_ = create_service<autoware_internal_debug_msgs::srv::String>(
    "~/service/load_plugin", std::bind(&BehaviorVelocityPlannerNode::onLoadPlugin, this, _1, _2));
  srv_unload_plugin_ = create_service<autoware_internal_debug_msgs::srv::String>(
    "~/service/unload_plugin",
    std::bind(&BehaviorVelocityPlannerNode::onUnloadPlugin, this, _1, _2));

  // set velocity smoother param
  onParam();

  // Publishers
  path_pub_ = this->create_publisher<autoware_planning_msgs::msg::Path>("~/output/path", 1);
  debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/path", 1);

  // Parameters
  forward_path_length_ = declare_parameter<double>("forward_path_length");
  backward_path_length_ = declare_parameter<double>("backward_path_length");
  behavior_output_path_interval_ = declare_parameter<double>("behavior_output_path_interval");
  planner_data_.stop_line_extend_length = declare_parameter<double>("stop_line_extend_length");

  // nearest search
  planner_data_.ego_nearest_dist_threshold =
    declare_parameter<double>("ego_nearest_dist_threshold");
  planner_data_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  // is simulation or not
  planner_data_.is_simulation = declare_parameter<bool>("is_simulation");

  // Initialize PlannerManager
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
    if (name == "") {
      break;
    }
    planner_manager_.launchScenePlugin(*this, name);
  }

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}

void BehaviorVelocityPlannerNode::onLoadPlugin(
  const autoware_internal_debug_msgs::srv::String::Request::SharedPtr request,
  [[maybe_unused]] const autoware_internal_debug_msgs::srv::String::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.launchScenePlugin(*this, request->data);
}

void BehaviorVelocityPlannerNode::onUnloadPlugin(
  const autoware_internal_debug_msgs::srv::String::Request::SharedPtr request,
  [[maybe_unused]] const autoware_internal_debug_msgs::srv::String::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.removeScenePlugin(*this, request->data);
}

void BehaviorVelocityPlannerNode::onParam()
{
  // Note(VRichardJP): mutex lock is not necessary as onParam is only called once in the
  // constructed. It would be required if it was a callback. std::lock_guard<std::mutex>
  // lock(mutex_);
  planner_data_.velocity_smoother_ =
    std::make_unique<autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother>(*this);
  planner_data_.velocity_smoother_->setWheelBase(planner_data_.vehicle_info_.wheel_base_m);
}

void BehaviorVelocityPlannerNode::processNoGroundPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  if (!pc.empty()) {
    autoware::universe_utils::transformPointCloud(pc, *pc_transformed, affine);
  }

  planner_data_.no_ground_pointcloud = pc_transformed;
}

void BehaviorVelocityPlannerNode::processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto current_odometry = std::make_shared<geometry_msgs::msg::PoseStamped>();
  current_odometry->header = msg->header;
  current_odometry->pose = msg->pose.pose;
  planner_data_.current_odometry = current_odometry;

  auto current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  current_velocity->header = msg->header;
  current_velocity->twist = msg->twist.twist;
  planner_data_.current_velocity = current_velocity;

  // Add velocity to buffer
  planner_data_.velocity_buffer.push_front(*current_velocity);
  const rclcpp::Time now = this->now();
  while (!planner_data_.velocity_buffer.empty()) {
    // Check oldest data time
    const auto & s = planner_data_.velocity_buffer.back().header.stamp;
    const auto time_diff =
      now >= s ? now - s : rclcpp::Duration(0, 0);  // Note: negative time throws an exception.

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    planner_data_.velocity_buffer.pop_back();
  }
}

void BehaviorVelocityPlannerNode::processTrafficSignals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg)
{
  // clear previous observation
  planner_data_.traffic_light_id_map_raw_.clear();
  const auto traffic_light_id_map_last_observed_old =
    planner_data_.traffic_light_id_map_last_observed_;
  planner_data_.traffic_light_id_map_last_observed_.clear();
  for (const auto & signal : msg->traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg->stamp;
    traffic_signal.signal = signal;
    planner_data_.traffic_light_id_map_raw_[signal.traffic_light_group_id] = traffic_signal;
    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      });
    // if the observation is UNKNOWN and past observation is available, only update the timestamp
    // and keep the body of the info
    const auto old_data =
      traffic_light_id_map_last_observed_old.find(signal.traffic_light_group_id);
    if (is_unknown_observation && old_data != traffic_light_id_map_last_observed_old.end()) {
      // copy last observation
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        old_data->second;
      // update timestamp
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id].stamp =
        msg->stamp;
    } else {
      // if (1)the observation is not UNKNOWN or (2)the very first observation is UNKNOWN
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        traffic_signal;
    }
  }
}

bool BehaviorVelocityPlannerNode::processData(rclcpp::Clock clock)
{
  bool is_ready = true;
  const auto & logData = [&clock, this](const std::string & data_type) {
    std::string msg = "Waiting for " + data_type + " data";
    RCLCPP_INFO_THROTTLE(get_logger(), clock, logger_throttle_interval, "%s", msg.c_str());
  };

  const auto & getData = [&logData](auto & dest, auto & sub, const std::string & data_type = "") {
    const auto temp = sub.takeData();
    if (temp) {
      dest = temp;
      return true;
    }
    if (!data_type.empty()) logData(data_type);
    return false;
  };

  is_ready &= getData(planner_data_.current_acceleration, sub_acceleration_, "acceleration");
  is_ready &= getData(planner_data_.predicted_objects, sub_predicted_objects_, "predicted_objects");
  is_ready &= getData(planner_data_.occupancy_grid, sub_occupancy_grid_, "occupancy_grid");

  const auto odometry = sub_vehicle_odometry_.takeData();
  if (odometry) {
    processOdometry(odometry);
  } else {
    logData("odometry");
    is_ready = false;
  }

  const auto no_ground_pointcloud = sub_no_ground_pointcloud_.takeData();
  if (no_ground_pointcloud) {
    processNoGroundPointCloud(no_ground_pointcloud);
  } else {
    logData("pointcloud");
    is_ready = false;
  }

  const auto map_data = sub_lanelet_map_.takeData();
  if (map_data) {
    planner_data_.route_handler_ = std::make_shared<route_handler::RouteHandler>(*map_data);
  }

  // planner_data_.external_velocity_limit is std::optional type variable.
  const auto external_velocity_limit = sub_external_velocity_limit_.takeData();
  if (external_velocity_limit) {
    planner_data_.external_velocity_limit = *external_velocity_limit;
  }

  const auto traffic_signals = sub_traffic_signals_.takeData();
  if (traffic_signals) processTrafficSignals(traffic_signals);

  return is_ready;
}

// NOTE: argument planner_data must not be referenced for multithreading
bool BehaviorVelocityPlannerNode::isDataReady(rclcpp::Clock clock)
{
  if (!planner_data_.velocity_smoother_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, logger_throttle_interval,
      "Waiting for the initialization of velocity smoother");
    return false;
  }

  return processData(clock);
}

void BehaviorVelocityPlannerNode::onTrigger(
  const tier4_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
{
  std::unique_lock<std::mutex> lk(mutex_);

  if (!isDataReady(*get_clock())) {
    return;
  }

  // Load map and check route handler
  if (!planner_data_.route_handler_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval,
      "Waiting for the initialization of route_handler");
    return;
  }

  if (input_path_msg->points.empty()) {
    return;
  }

  const autoware_planning_msgs::msg::Path output_path_msg =
    generatePath(input_path_msg, planner_data_);

  lk.unlock();

  path_pub_->publish(output_path_msg);
  published_time_publisher_->publish_if_subscribed(path_pub_, output_path_msg.header.stamp);

  if (debug_viz_pub_->get_subscription_count() > 0) {
    publishDebugMarker(output_path_msg);
  }
}

autoware_planning_msgs::msg::Path BehaviorVelocityPlannerNode::generatePath(
  const tier4_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
  const PlannerData & planner_data)
{
  autoware_planning_msgs::msg::Path output_path_msg;

  // TODO(someone): support backward path
  const auto is_driving_forward = autoware::motion_utils::isDrivingForward(input_path_msg->points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval,
      "Backward path is NOT supported. just converting path_with_lane_id to path");
    output_path_msg = to_path(*input_path_msg);
    output_path_msg.header.frame_id = "map";
    output_path_msg.header.stamp = this->now();
    output_path_msg.left_bound = input_path_msg->left_bound;
    output_path_msg.right_bound = input_path_msg->right_bound;
    return output_path_msg;
  }

  // Plan path velocity
  const auto velocity_planned_path = planner_manager_.planPathVelocity(
    std::make_shared<const PlannerData>(planner_data), *input_path_msg);

  // screening
  const auto filtered_path =
    autoware::behavior_velocity_planner::filterLitterPathPoint(to_path(velocity_planned_path));

  // interpolation
  const auto interpolated_path_msg = autoware::behavior_velocity_planner::interpolatePath(
    filtered_path, forward_path_length_, behavior_output_path_interval_);

  // check stop point
  output_path_msg = autoware::behavior_velocity_planner::filterStopPathPoint(interpolated_path_msg);

  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = this->now();

  // TODO(someone): This must be updated in each scene module, but copy from input message for now.
  output_path_msg.left_bound = input_path_msg->left_bound;
  output_path_msg.right_bound = input_path_msg->right_bound;

  return output_path_msg;
}

void BehaviorVelocityPlannerNode::publishDebugMarker(const autoware_planning_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    output_msg.markers.push_back(marker);
  }
  debug_viz_pub_->publish(output_msg);
}
}  // namespace autoware::behavior_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode)
