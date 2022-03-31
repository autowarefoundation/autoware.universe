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

#include <lanelet2_extension/utility/query.hpp>
#include <scene_module/dynamic_obstacle_stop/manager.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
}  // namespace

DynamicObstacleStopModuleManager::DynamicObstacleStopModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  // Vehicle Parameters
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
    auto & p = planner_param_.vehicle_param;
    p.base_to_front = vehicle_info.wheel_base_m + vehicle_info.front_overhang_m;
    p.base_to_rear = vehicle_info.rear_overhang_m;
    p.width = vehicle_info.vehicle_width_m;
  }

  const std::string ns(getModuleName());

  {
    auto & p = planner_param_.common;
    p.normal_min_jerk = node.declare_parameter(".normal.min_jerk", -0.3);
    p.normal_min_acc = node.declare_parameter(".normal.min_acc", -1.0);
    p.limit_min_jerk = node.declare_parameter(".limit.min_jerk", -1.5);
    p.limit_min_acc = node.declare_parameter(".limit.min_acc", -2.5);
  }

  {
    auto & p = planner_param_.dynamic_obstacle_stop;
    p.enable_dynamic_obstacle_stop =
      node.declare_parameter(ns + ".enable_dynamic_obstacle_stop", true);
    p.use_objects = node.declare_parameter(ns + ".use_objects", true);
    p.use_predicted_path = node.declare_parameter(ns + ".use_predicted_path", false);
    p.extend_distance = node.declare_parameter(ns + ".extend_distance", 5.0);
    p.stop_margin = node.declare_parameter(ns + ".stop_margin", 2.5);
    p.passing_margin = node.declare_parameter(ns + ".passing_margin", 1.0);
    p.stop_start_jerk_dec = node.declare_parameter(ns + ".stop_start_jerk_dec", -0.3);
    p.obstacle_velocity_kph = node.declare_parameter(ns + ".obstacle_velocity_kph", 5.0);
    p.detection_distance = node.declare_parameter(ns + ".detection_distance", 45.0);
    p.detection_span = node.declare_parameter(ns + ".detection_span", 1.0);
    p.min_vel_ego_kmph = node.declare_parameter(ns + ".min_vel_ego_kmph", 5.0);
    p.calc_collision_from_point = node.declare_parameter(ns + ".calc_collision_from_point", true);
  }

  {
    auto & p = planner_param_.detection_area;
    const std::string ns_da = ns + ".detection_area_size";
    p.dist_ahead = node.declare_parameter(ns_da + ".dist_ahead", 50.0);
    p.dist_behind = node.declare_parameter(ns_da + ".dist_behind", 5.0);
    p.dist_right = node.declare_parameter(ns_da + ".dist_right", 10.0);
    p.dist_left = node.declare_parameter(ns_da + ".dist_left", 10.0);
  }

  {
    auto & p = planner_param_.dynamic_obstacle;
    const std::string ns_do = ns + ".dynamic_obstacle";
    p.min_vel_kmph = node.declare_parameter(ns_do + ".min_vel_kmph", 0.0);
    p.max_vel_kmph = node.declare_parameter(ns_do + ".max_vel_kmph", 5.0);
    p.diameter = node.declare_parameter(ns_do + ".diameter", 0.1);
    p.height = node.declare_parameter(ns_do + ".height", 2.0);
    p.path_size = node.declare_parameter(ns_do + ".path_size", 20);
    p.time_step = node.declare_parameter(ns_do + ".time_step", 0.5);
  }

  {
    auto & p = planner_param_.approaching;
    const std::string ns_a = ns + "approaching";
    p.enable = node.declare_parameter(ns_a + ".enable", false);
    p.margin = node.declare_parameter(ns_a + ".margin", 0.0);
    p.limit_vel_kmph = node.declare_parameter(ns_a + ".limit_vel_kmph", 5.0);
    p.stop_thresh = node.declare_parameter(ns_a + ".stop_thresh", 0.01);
    p.stop_time_thresh = node.declare_parameter(ns_a + ".stop_time_thresh", 3.0);
    p.dist_thresh = node.declare_parameter(ns_a + ".dist_thresh", 0.5);
  }

  initSmootherParam(node);

  // // Set parameter callback
  // set_param_res_ = this->add_on_set_parameters_callback(
  //   std::bind(&DynamicObstacleStopPlannerNode::paramCallback, this, std::placeholders::_1));

  // Initializer
  debug_ptr_ = std::make_shared<DynamicObstacleStopDebug>(node);

  // // Publishers
  // path_pub_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);
  // // stop_reason_diag_pub_ =
  // //   this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/stop_reason", 1);
  // debug_path_pub_ = this->create_publisher<Trajectory>("~/debug/trajectory", 1);
  // debug_value_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/debug_value", 1);
  // lateral_distance_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/lateral_distance", 1);
  // longitudinal_distance_obstacle_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
  //     "~/debug/longitudinal_distance_obstacle", 1);
  // longitudinal_distance_collision_point_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
  //     "~/debug/longitudinal_distance_collision_point", 1);
  // vehicle_side_collision_dist_pub_ =
  // this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
  //   "~/debug/vehicle_side_collision_dist", 1);
  // dist_to_collision_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/dist_to_collision",
  //   1);
  // stop_dist_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/stop_dist", 1);
  // insert_vel_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/insert_vel", 1);
  // calculation_time_pub_ =
  //   this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/calculation_time", 1);

  // // Subscribers
  // obstacle_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "~/input/pointcloud", rclcpp::SensorDataQoS(),
  //   std::bind(
  //     &DynamicObstacleStopPlannerNode::obstaclePointcloudCallback, this, std::placeholders::_1));
  // path_sub_ = this->create_subscription<Trajectory>(
  //   "~/input/trajectory", 1,
  //   std::bind(&DynamicObstacleStopPlannerNode::pathCallback, this, std::placeholders::_1));
  // current_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "~/input/odometry", 1,
  //   std::bind(
  //     &DynamicObstacleStopPlannerNode::currentVelocityCallback, this, std::placeholders::_1));
  // dynamic_object_sub_ = this->create_subscription<PredictedObjects>(
  //   "~/input/objects", 1,
  //   std::bind(&DynamicObstacleStopPlannerNode::dynamicObjectCallback, this,
  //   std::placeholders::_1));

  // // TODO(Tomohito Ando): temporary for experiments
  // smoothed_trajectory_sub_ = this->create_subscription<Trajectory>(
  //   "/planning/scenario_planning/trajectory", 1,
  //   std::bind(
  //     &DynamicObstacleStopPlannerNode::smoothedTrajectoryCallback, this, std::placeholders::_1));
  // velocity_limit_sub_ = this->create_subscription<tier4_planning_msgs::msg::VelocityLimit>(
  //   "/planning/scenario_planning/max_velocity", 1,
  //   std::bind(&DynamicObstacleStopPlannerNode::velocityLimitCallback, this,
  //   std::placeholders::_1));
}

void DynamicObstacleStopModuleManager::initSmootherParam(rclcpp::Node & node)
{
  motion_velocity_smoother::SmootherBase::BaseParam base_param;
  {
    auto & p = base_param;
    p.min_decel = planner_param_.common.normal_min_acc;
    p.min_jerk = planner_param_.common.normal_min_jerk;
    p.max_accel = node.declare_parameter("normal.max_acc", 2.0);
    p.max_jerk = node.declare_parameter("normal.max_jerk", 0.3);
    p.stop_decel = node.declare_parameter("stop_decel", 0.0);
    p.max_lateral_accel = node.declare_parameter("max_lateral_accel", 0.2);
    p.decel_distance_before_curve = node.declare_parameter("decel_distance_before_curve", 3.5);
    p.decel_distance_after_curve = node.declare_parameter("decel_distance_after_curve", 0.0);
    p.min_curve_velocity = node.declare_parameter("min_curve_velocity", 1.38);
    p.resample_param.max_trajectory_length = node.declare_parameter("max_trajectory_length", 200.0);
    p.resample_param.min_trajectory_length = node.declare_parameter("min_trajectory_length", 30.0);
    p.resample_param.resample_time = node.declare_parameter("resample_time", 10.0);
    p.resample_param.dense_resample_dt = node.declare_parameter("dense_resample_dt", 0.1);
    p.resample_param.dense_min_interval_distance =
      node.declare_parameter("dense_min_interval_distance", 0.1);
    p.resample_param.sparse_resample_dt = node.declare_parameter("sparse_resample_dt", 0.5);
    p.resample_param.sparse_min_interval_distance =
      node.declare_parameter("sparse_min_interval_distance", 4.0);
  }

  motion_velocity_smoother::AnalyticalJerkConstrainedSmoother::Param
    analytical_jerk_constrained_smoother_param;
  {
    auto & p = analytical_jerk_constrained_smoother_param;
    p.resample.ds_resample = node.declare_parameter("resample.ds_resample", 0.1);
    p.resample.num_resample = node.declare_parameter("resample.num_resample", 1);
    p.resample.delta_yaw_threshold = node.declare_parameter("resample.delta_yaw_threshold", 0.785);

    p.latacc.enable_constant_velocity_while_turning =
      node.declare_parameter("latacc.enable_constant_velocity_while_turning", false);
    p.latacc.constant_velocity_dist_threshold =
      node.declare_parameter("latacc.constant_velocity_dist_threshold", 2.0);

    p.forward.max_acc = node.declare_parameter("forward.max_acc", 1.0);
    p.forward.min_acc = node.declare_parameter("forward.min_acc", -1.0);
    p.forward.max_jerk = node.declare_parameter("forward.max_jerk", 0.3);
    p.forward.min_jerk = node.declare_parameter("forward.min_jerk", -0.3);
    p.forward.kp = node.declare_parameter("forward.kp", 0.3);

    p.backward.start_jerk = node.declare_parameter("backward.start_jerk", -0.1);
    p.backward.min_jerk_mild_stop = node.declare_parameter("backward.min_jerk_mild_stop", -0.3);
    p.backward.min_jerk = node.declare_parameter("backward.min_jerk", -1.5);
    p.backward.min_acc_mild_stop = node.declare_parameter("backward.min_acc_mild_stop", -1.0);
    p.backward.min_acc = node.declare_parameter("backward.min_acc", -2.5);
    p.backward.span_jerk = node.declare_parameter("backward.span_jerk", -0.01);
  }

  smoother_ = std::make_shared<motion_velocity_smoother::AnalyticalJerkConstrainedSmoother>(
    analytical_jerk_constrained_smoother_param);
  smoother_->setParam(base_param);
}

void DynamicObstacleStopModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) {
    return;
  }

  // TODO(Tomohito Ando): appropreate id
  constexpr int64_t module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(std::make_shared<DynamicObstacleStopModule>(
      module_id, planner_param_, logger_.get_child("dynamic_obstacle_stop_module"), smoother_,
      debug_ptr_, clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
DynamicObstacleStopModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  return
    [&path]([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) -> bool {
      return false;
    };
}
}  // namespace behavior_velocity_planner
