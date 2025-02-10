// Copyright 2025 Tier IV, Inc.
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

#include "autoware/control_evaluator/control_evaluator_node.hpp"

#include "autoware/control_evaluator/metrics/metrics_utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace control_diagnostics
{
ControlEvaluatorNode::ControlEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("control_evaluator", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  // Publisher
  processing_time_pub_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
  metrics_pub_ = create_publisher<MetricArrayMsg>("~/metrics", 1);

  // Parameters
  output_metrics_ = declare_parameter<bool>("output_metrics");

  // Timer callback to publish evaluator diagnostics
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&ControlEvaluatorNode::onTimer, this));
}

ControlEvaluatorNode::~ControlEvaluatorNode()
{
  if (!output_metrics_) {
    return;
  }

  try {
    // generate json data
    using json = nlohmann::json;
    json j;
    for (Metric metric : metrics_) {
      const std::string base_name = metric_to_str.at(metric) + "/";
      j[base_name + "min"] = metric_accumulators_[static_cast<size_t>(metric)].min();
      j[base_name + "max"] = metric_accumulators_[static_cast<size_t>(metric)].max();
      j[base_name + "mean"] = metric_accumulators_[static_cast<size_t>(metric)].mean();
      j[base_name + "count"] = metric_accumulators_[static_cast<size_t>(metric)].count();
      j[base_name + "description"] = metric_descriptions.at(metric);
    }

    // get output folder
    const std::string output_folder_str =
      rclcpp::get_logging_directory().string() + "/autoware_metrics";
    if (!std::filesystem::exists(output_folder_str)) {
      if (!std::filesystem::create_directories(output_folder_str)) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to create directories: %s", output_folder_str.c_str());
        return;
      }
    }

    // get time stamp
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm * local_time = std::localtime(&now_time_t);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y-%m-%d-%H-%M-%S");
    std::string cur_time_str = oss.str();

    // Write metrics .json to file
    const std::string output_file_str =
      output_folder_str + "/autoware_control_evaluator-" + cur_time_str + ".json";
    std::ofstream f(output_file_str);
    if (f.is_open()) {
      f << j.dump(4);
      f.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", output_file_str.c_str());
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in ControlEvaluatorNode destructor: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception in ControlEvaluatorNode destructor" << std::endl;
  }
}

void ControlEvaluatorNode::getRouteData()
{
  // route
  {
    const auto msg = route_subscriber_.takeData();
    if (msg) {
      if (msg->segments.empty()) {
        RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
      } else {
        route_handler_.setRoute(*msg);
      }
    }
  }

  // map
  {
    const auto msg = vector_map_subscriber_.takeData();
    if (msg) {
      route_handler_.setMap(*msg);
    }
  }
}

void ControlEvaluatorNode::AddMetricMsg(const Metric & metric, const double & metric_value)
{
  MetricMsg metric_msg;
  metric_msg.name = metric_to_str.at(metric);
  metric_msg.value = std::to_string(metric_value);
  metrics_msg_.metric_array.push_back(metric_msg);

  if (output_metrics_) {
    metric_accumulators_[static_cast<size_t>(metric)].add(metric_value);
  }
}

void ControlEvaluatorNode::AddLaneletInfoMsg(const Pose & ego_pose)
{
  const auto current_lanelets = metrics::utils::get_current_lanes(route_handler_, ego_pose);
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(current_lanelets, ego_pose);
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanelets, ego_pose, &current_lane);

  const std::string base_name = "ego_lane_info/";
  MetricMsg metric_msg;

  {
    metric_msg.name = base_name + "lane_id";
    metric_msg.value = std::to_string(current_lane.id());
    metrics_msg_.metric_array.push_back(metric_msg);
  }

  {
    metric_msg.name = base_name + "s";
    metric_msg.value = std::to_string(arc_coordinates.length);
    metrics_msg_.metric_array.push_back(metric_msg);
  }

  {
    metric_msg.name = base_name + "t";
    metric_msg.value = std::to_string(arc_coordinates.distance);
    metrics_msg_.metric_array.push_back(metric_msg);
  }
}

void ControlEvaluatorNode::AddBoundaryDistanceMetricMsg(
  const PathWithLaneId & behavior_path, const Pose & ego_pose)
{
  const auto current_lanelets = metrics::utils::get_current_lanes(route_handler_, ego_pose);
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanelets, ego_pose, &current_lane);
  const auto local_vehicle_footprint = vehicle_info_.createFootprint();
  const auto current_vehicle_footprint =
    transformVector(local_vehicle_footprint, autoware::universe_utils::pose2transform(ego_pose));

  if (behavior_path.left_bound.size() >= 1) {
    LineString2d left_boundary;
    for (const auto & p : behavior_path.left_bound) left_boundary.push_back(Point2d(p.x, p.y));
    double distance_to_left_boundary =
      metrics::utils::calc_distance_to_line(current_vehicle_footprint, left_boundary);

    if (metrics::utils::is_point_left_of_line(ego_pose.position, behavior_path.left_bound)) {
      distance_to_left_boundary *= -1.0;
    }
    const Metric metric_left = Metric::left_boundary_distance;
    AddMetricMsg(metric_left, distance_to_left_boundary);
  }

  if (behavior_path.right_bound.size() >= 1) {
    LineString2d right_boundary;
    for (const auto & p : behavior_path.right_bound) right_boundary.push_back(Point2d(p.x, p.y));
    double distance_to_right_boundary =
      metrics::utils::calc_distance_to_line(current_vehicle_footprint, right_boundary);

    if (!metrics::utils::is_point_left_of_line(ego_pose.position, behavior_path.right_bound)) {
      distance_to_right_boundary *= -1.0;
    }
    const Metric metric_right = Metric::right_boundary_distance;
    AddMetricMsg(metric_right, distance_to_right_boundary);
  }
}

void ControlEvaluatorNode::AddKinematicStateMetricMsg(
  const Odometry & odom, const AccelWithCovarianceStamped & accel_stamped)
{
  const std::string base_name = "kinematic_state/";
  MetricMsg metric_msg;

  metric_msg.name = base_name + "vel";
  metric_msg.value = std::to_string(odom.twist.twist.linear.x);
  metrics_msg_.metric_array.push_back(metric_msg);

  metric_msg.name = base_name + "acc";
  const auto & acc = accel_stamped.accel.accel.linear.x;
  metric_msg.value = std::to_string(acc);
  metrics_msg_.metric_array.push_back(metric_msg);

  const auto jerk = [&]() {
    if (!prev_acc_stamped_.has_value()) {
      prev_acc_stamped_ = accel_stamped;
      return 0.0;
    }
    const auto t = static_cast<double>(accel_stamped.header.stamp.sec) +
                   static_cast<double>(accel_stamped.header.stamp.nanosec) * 1e-9;
    const auto prev_t = static_cast<double>(prev_acc_stamped_.value().header.stamp.sec) +
                        static_cast<double>(prev_acc_stamped_.value().header.stamp.nanosec) * 1e-9;
    const auto dt = t - prev_t;
    if (dt < std::numeric_limits<double>::epsilon()) return 0.0;

    const auto prev_acc = prev_acc_stamped_.value().accel.accel.linear.x;
    prev_acc_stamped_ = accel_stamped;
    return (acc - prev_acc) / dt;
  }();

  metric_msg.name = base_name + "jerk";
  metric_msg.value = std::to_string(jerk);
  metrics_msg_.metric_array.push_back(metric_msg);
  return;
}

void ControlEvaluatorNode::AddSteeringMetricMsg(const SteeringReport & steering_status)
{
  // steering angle
  double cur_steering_angle = steering_status.steering_tire_angle;
  const double cur_t = static_cast<double>(steering_status.stamp.sec) +
                       static_cast<double>(steering_status.stamp.nanosec) * 1e-9;
  AddMetricMsg(Metric::steering_angle, cur_steering_angle);

  if (!prev_steering_angle_timestamp_.has_value()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    return;
  }

  // d_t
  const double dt = cur_t - prev_steering_angle_timestamp_.value();
  if (dt < std::numeric_limits<double>::epsilon()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    return;
  }

  // steering rate
  const double steering_rate = (cur_steering_angle - prev_steering_angle_.value()) / dt;
  AddMetricMsg(Metric::steering_rate, steering_rate);

  // steering acceleration
  if (!prev_steering_rate_.has_value()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    prev_steering_rate_ = steering_rate;
    return;
  }
  const double steering_acceleration = (steering_rate - prev_steering_rate_.value()) / dt;
  AddMetricMsg(Metric::steering_acceleration, steering_acceleration);

  prev_steering_angle_timestamp_ = cur_t;
  prev_steering_angle_ = cur_steering_angle;
  prev_steering_rate_ = steering_rate;
}

void ControlEvaluatorNode::AddLateralDeviationMetricMsg(
  const Trajectory & traj, const Point & ego_point)
{
  const Metric metric = Metric::lateral_deviation;
  const double metric_value = metrics::calcLateralDeviation(traj, ego_point);

  AddMetricMsg(metric, metric_value);
}

void ControlEvaluatorNode::AddYawDeviationMetricMsg(const Trajectory & traj, const Pose & ego_pose)
{
  const Metric metric = Metric::yaw_deviation;
  const double metric_value = metrics::calcYawDeviation(traj, ego_pose);

  AddMetricMsg(metric, metric_value);
}

void ControlEvaluatorNode::AddGoalLongitudinalDeviationMetricMsg(const Pose & ego_pose)
{
  const Metric metric = Metric::goal_longitudinal_deviation;
  const double metric_value =
    metrics::calcLongitudinalDeviation(route_handler_.getGoalPose(), ego_pose.position);

  AddMetricMsg(metric, metric_value);
}

void ControlEvaluatorNode::AddGoalLateralDeviationMetricMsg(const Pose & ego_pose)
{
  const Metric metric = Metric::goal_lateral_deviation;
  const double metric_value =
    metrics::calcLateralDeviation(route_handler_.getGoalPose(), ego_pose.position);

  AddMetricMsg(metric, metric_value);
}

void ControlEvaluatorNode::AddGoalYawDeviationMetricMsg(const Pose & ego_pose)
{
  const Metric metric = Metric::goal_yaw_deviation;
  const double metric_value = metrics::calcYawDeviation(route_handler_.getGoalPose(), ego_pose);

  AddMetricMsg(metric, metric_value);
}

void ControlEvaluatorNode::onTimer()
{
  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  const auto traj = traj_sub_.takeData();
  const auto odom = odometry_sub_.takeData();
  const auto acc = accel_sub_.takeData();
  const auto behavior_path = behavior_path_subscriber_.takeData();
  const auto steering_status = steering_sub_.takeData();

  // calculate deviation metrics
  if (odom && traj && !traj->points.empty()) {
    const Pose ego_pose = odom->pose.pose;
    AddLateralDeviationMetricMsg(*traj, ego_pose.position);
    AddYawDeviationMetricMsg(*traj, ego_pose);
  }

  getRouteData();
  if (odom && route_handler_.isHandlerReady()) {
    const Pose ego_pose = odom->pose.pose;
    AddLaneletInfoMsg(ego_pose);
    AddGoalLongitudinalDeviationMetricMsg(ego_pose);
    AddGoalLateralDeviationMetricMsg(ego_pose);
    AddGoalYawDeviationMetricMsg(ego_pose);
  }

  if (odom && acc) {
    AddKinematicStateMetricMsg(*odom, *acc);
  }

  if (odom && behavior_path) {
    const Pose ego_pose = odom->pose.pose;
    AddBoundaryDistanceMetricMsg(*behavior_path, ego_pose);
  }

  if (steering_status) {
    AddSteeringMetricMsg(*steering_status);
  }

  // Publish metrics
  metrics_msg_.stamp = now();
  metrics_pub_->publish(metrics_msg_);
  metrics_msg_ = MetricArrayMsg{};

  // Publish processing time
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}
}  // namespace control_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(control_diagnostics::ControlEvaluatorNode)
