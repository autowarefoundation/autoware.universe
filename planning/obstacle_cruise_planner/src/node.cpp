// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_cruise_planner/node.hpp"

#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/tmp_conversion.hpp"
#include "obstacle_cruise_planner/polygon_utils.hpp"
#include "obstacle_cruise_planner/utils.hpp"
#include "perception_utils/predicted_path_utils.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <boost/format.hpp>

#include <algorithm>
#include <chrono>

namespace
{
VelocityLimitClearCommand createVelocityLimitClearCommandMessage(
  const rclcpp::Time & current_time, const std::string & module_name)
{
  VelocityLimitClearCommand msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise_planner." + module_name;
  msg.command = true;
  return msg;
}

template <typename T>
std::optional<T> getObstacleFromId(
  const std::vector<T> & obstacles, const std::string & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}

bool isFrontObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos)
{
  size_t obstacle_idx = motion_utils::findNearestSegmentIndex(traj_points, obstacle_pos);

  const double ego_to_obstacle_distance =
    motion_utils::calcSignedArcLength(traj_points, ego_idx, obstacle_idx);

  if (ego_to_obstacle_distance < 0) {
    return false;
  }

  return true;
}

PredictedPath getHighestConfidencePredictedPath(const std::vector<PredictedPath> & predicted_paths)
{
  const auto reliable_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });
  return *reliable_path;
}

bool isAngleAlignedWithTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose,
  const double threshold_angle)
{
  if (traj_points.empty()) {
    return false;
  }

  const double obstacle_yaw = tf2::getYaw(pose.orientation);

  const size_t nearest_idx = motion_utils::findNearestIndex(traj_points, pose.position);
  const double traj_yaw = tf2::getYaw(traj_points.at(nearest_idx).pose.orientation);

  const double diff_yaw = tier4_autoware_utils::normalizeRadian(obstacle_yaw - traj_yaw);

  // TODO(perception team) Currently predicted objects does not have orientation availability even
  // though sometimes orientation is not available.
  const bool is_aligned =
    std::abs(diff_yaw) <= threshold_angle || std::abs(M_PI - std::abs(diff_yaw)) <= threshold_angle;
  return is_aligned;
}

double calcObstacleProjectedVelocity(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle)
{
  const size_t object_idx = motion_utils::findNearestIndex(traj_points, obstacle.pose.position);

  const double object_yaw = tf2::getYaw(obstacle.pose.orientation);
  const double traj_yaw = tf2::getYaw(traj_points.at(object_idx).pose.orientation);

  return obstacle.twist.linear.x * std::cos(object_yaw - traj_yaw);
}

double calcObstacleMaxLength(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point)
{
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose =
    tier4_autoware_utils::calcOffsetPose(goal_point.pose, extend_distance, 0.0, 0.0);
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

std::vector<TrajectoryPoint> extendTrajectoryPoints(
  const std::vector<TrajectoryPoint> & input_points, const double extend_distance,
  const double step_length)
{
  auto output_points = input_points;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output_points;
  }

  const auto goal_point = input_points.back();

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - step_length)) {
    const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_sum, goal_point);
    output_points.push_back(extend_trajectory_point);
    extend_sum += step_length;
  }
  const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_distance, goal_point);
  output_points.push_back(extend_trajectory_point);

  return output_points;
}

Trajectory createTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const std_msgs::msg::Header & header)
{
  auto traj = motion_utils::convertToTrajectory(traj_points);
  traj.header = header;

  return traj;
}

std::vector<int> getTargetObjectType(rclcpp::Node & node, const std::string & param_prefix)
{
  std::unordered_map<std::string, int> types_map{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};

  std::vector<int> types;
  for (const auto & type : types_map) {
    if (node.declare_parameter<bool>(param_prefix + type.first)) {
      types.push_back(type.second);
    }
  }
  return types;
}
}  // namespace

namespace motion_planning
{
ObstacleCruisePlannerNode::ObstacleFilteringParam::ObstacleFilteringParam(rclcpp::Node & node)
{  // Obstacle filtering parameters
  rough_detection_area_expand_width =
    node.declare_parameter<double>("obstacle_filtering.rough_detection_area_expand_width");
  decimate_trajectory_step_length =
    node.declare_parameter<double>("obstacle_filtering.decimate_trajectory_step_length");
  crossing_obstacle_velocity_threshold =
    node.declare_parameter<double>("obstacle_filtering.crossing_obstacle_velocity_threshold");
  collision_time_margin =
    node.declare_parameter<double>("obstacle_filtering.collision_time_margin");
  outside_rough_detection_area_expand_width =
    node.declare_parameter<double>("obstacle_filtering.outside_rough_detection_area_expand_width");
  outside_obstacle_min_velocity_threshold =
    node.declare_parameter<double>("obstacle_filtering.outside_obstacle_min_velocity_threshold");
  ego_obstacle_overlap_time_threshold =
    node.declare_parameter<double>("obstacle_filtering.ego_obstacle_overlap_time_threshold");
  max_prediction_time_for_collision_check =
    node.declare_parameter<double>("obstacle_filtering.max_prediction_time_for_collision_check");
  crossing_obstacle_traj_angle_threshold =
    node.declare_parameter<double>("obstacle_filtering.crossing_obstacle_traj_angle_threshold");
  stop_obstacle_hold_time_threshold =
    node.declare_parameter<double>("obstacle_filtering.stop_obstacle_hold_time_threshold");
  prediction_resampling_time_interval =
    node.declare_parameter<double>("obstacle_filtering.prediction_resampling_time_interval");
  prediction_resampling_time_horizon =
    node.declare_parameter<double>("obstacle_filtering.prediction_resampling_time_horizon");
  goal_extension_length =
    node.declare_parameter<double>("obstacle_filtering.goal_extension_length");
  goal_extension_interval =
    node.declare_parameter<double>("obstacle_filtering.goal_extension_interval");

  ignored_outside_obstacle_types =
    getTargetObjectType(node, "obstacle_filtering.ignored_outside_obstacle_type.");
}

void ObstacleCruisePlannerNode::ObstacleFilteringParam::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // obstacle_filtering
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.rough_detection_area_expand_width",
    rough_detection_area_expand_width);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.decimate_trajectory_step_length",
    decimate_trajectory_step_length);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.crossing_obstacle_velocity_threshold",
    crossing_obstacle_velocity_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.collision_time_margin", collision_time_margin);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.outside_rough_detection_area_expand_width",
    outside_rough_detection_area_expand_width);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.ego_obstacle_overlap_time_threshold",
    ego_obstacle_overlap_time_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.max_prediction_time_for_collision_check",
    max_prediction_time_for_collision_check);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.crossing_obstacle_traj_angle_threshold",
    crossing_obstacle_traj_angle_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.stop_obstacle_hold_time_threshold",
    stop_obstacle_hold_time_threshold);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.prediction_resampling_time_interval",
    prediction_resampling_time_interval);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.prediction_resampling_time_horizon",
    prediction_resampling_time_horizon);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.goal_extension_length", goal_extension_length);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.goal_extension_interval", goal_extension_interval);
}

ObstacleCruisePlannerNode::ObstacleCruisePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_cruise_planner", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  // subscriber
  traj_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleCruisePlannerNode::onTrajectory, this, _1));
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    [this](const PredictedObjects::ConstSharedPtr msg) { objects_ptr_ = msg; });
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    [this](const Odometry::ConstSharedPtr msg) { ego_odom_ptr_ = msg; });
  acc_sub_ = create_subscription<AccelWithCovarianceStamped>(
    "~/input/acceleration", rclcpp::QoS{1},
    [this](const AccelWithCovarianceStamped::ConstSharedPtr msg) { ego_accel_ptr_ = msg; });

  // publisher
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  vel_limit_pub_ =
    create_publisher<VelocityLimit>("~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
  clear_vel_limit_pub_ = create_publisher<VelocityLimitClearCommand>(
    "~/output/clear_velocity_limit", rclcpp::QoS{1}.transient_local());

  // debug publisher
  debug_calculation_time_pub_ = create_publisher<Float32Stamped>("~/debug/calculation_time", 1);
  debug_cruise_wall_marker_pub_ = create_publisher<MarkerArray>("~/debug/cruise/virtual_wall", 1);
  debug_stop_wall_marker_pub_ = create_publisher<MarkerArray>("~/virtual_wall", 1);
  debug_marker_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_stop_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/stop_planning_info", 1);
  debug_cruise_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/cruise_planning_info", 1);

  const auto longitudinal_info = LongitudinalInfo(*this);

  ego_nearest_param_ = EgoNearestParam(*this);

  is_showing_debug_info_ = declare_parameter<bool>("common.is_showing_debug_info");
  disable_stop_planning_ = declare_parameter<bool>("common.disable_stop_planning");
  enable_slow_down_planning_ = declare_parameter<bool>("common.enable_slow_down_planning");

  obstacle_filtering_param_ = ObstacleFilteringParam(*this);

  {  // planning algorithm
    const std::string planning_algorithm_param =
      declare_parameter<std::string>("common.planning_algorithm");
    planning_algorithm_ = getPlanningAlgorithmType(planning_algorithm_param);

    if (planning_algorithm_ == PlanningAlgorithm::OPTIMIZATION_BASE) {
      planner_ptr_ = std::make_unique<OptimizationBasedPlanner>(
        *this, longitudinal_info, vehicle_info_, ego_nearest_param_, debug_data_ptr_);
    } else if (planning_algorithm_ == PlanningAlgorithm::PID_BASE) {
      planner_ptr_ = std::make_unique<PIDBasedPlanner>(
        *this, longitudinal_info, vehicle_info_, ego_nearest_param_, debug_data_ptr_);
    } else {
      std::logic_error("Designated algorithm is not supported.");
    }

    min_behavior_stop_margin_ = declare_parameter<double>("common.min_behavior_stop_margin");
    obstacle_velocity_threshold_from_cruise_to_stop_ =
      declare_parameter<double>("common.obstacle_velocity_threshold_from_cruise_to_stop");
    obstacle_velocity_threshold_from_stop_to_cruise_ =
      declare_parameter<double>("common.obstacle_velocity_threshold_from_stop_to_cruise");
    planner_ptr_->setParam(is_showing_debug_info_, min_behavior_stop_margin_);
  }

  {  // stop/cruise/slow down obstacle type
    stop_obstacle_types_ = getTargetObjectType(*this, "common.stop_obstacle_type.");
    cruise_obstacle_types_ = getTargetObjectType(*this, "common.cruise_obstacle_type.");
    slow_down_obstacle_types_ = getTargetObjectType(*this, "common.slow_down_obstacle_type.");
  }

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleCruisePlannerNode::onParam, this, std::placeholders::_1));
}

ObstacleCruisePlannerNode::PlanningAlgorithm ObstacleCruisePlannerNode::getPlanningAlgorithmType(
  const std::string & param) const
{
  if (param == "pid_base") {
    return PlanningAlgorithm::PID_BASE;
  } else if (param == "optimization_base") {
    return PlanningAlgorithm::OPTIMIZATION_BASE;
  }
  return PlanningAlgorithm::INVALID;
}

rcl_interfaces::msg::SetParametersResult ObstacleCruisePlannerNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  planner_ptr_->onParam(parameters);

  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.is_showing_debug_info", is_showing_debug_info_);
  planner_ptr_->setParam(is_showing_debug_info_, min_behavior_stop_margin_);

  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.disable_stop_planning", disable_stop_planning_);
  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.enable_slow_down_planning", enable_slow_down_planning_);

  obstacle_filtering_param_.onParam(parameters);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ObstacleCruisePlannerNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  const auto traj_points = motion_utils::convertToTrajectoryPointArray(*msg);

  // check if subscribed variables are ready
  if (traj_points.empty() || !ego_odom_ptr_ || !ego_accel_ptr_ || !objects_ptr_) {
    return;
  }

  stop_watch_.tic(__func__);

  debug_data_ptr_ = std::make_shared<DebugData>();
  const auto is_driving_forward = motion_utils::isDrivingForwardWithTwist(traj_points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.get() : is_driving_forward_;

  // 1. Convert predicted objects to obstacles which are
  //    (1) in front of ego
  //    (2) with a proper label
  //    (3) and lateral deviation to trajectory
  const auto target_obstacles = convertToObstacles(traj_points);

  // 2. Determine ego's behavior against each obstacle from stop, cruise and slow down.
  const auto & [stop_obstacles, cruise_obstacles, slow_down_obstacles] =
    determineEgoBehaviorAgainstObstacles(traj_points, target_obstacles);

  // 3. Create data for planning
  const auto planner_data = createPlannerData(traj_points);

  // 4. Stop planning
  const auto stop_traj = planner_ptr_->generateStopTrajectory(planner_data, stop_obstacles);

  // 5. Cruise planning
  std::optional<VelocityLimit> cruise_vel_limit;
  const auto output_traj_points =
    planner_ptr_->generateCruiseTrajectory(planner_data, cruise_obstacles, cruise_vel_limit);
  publishVelocityLimit(cruise_vel_limit, "cruise");

  // 6. Slow down planning
  const auto slow_down_vel_limit =
    planner_ptr_->getSlowDownVelocityLimit(planner_data, slow_down_obstacles);
  publishVelocityLimit(slow_down_vel_limit, "slow_down");

  // 7. Publish trajectory
  const auto output_traj = createTrajectory(output_traj_points, msg->header);
  trajectory_pub_->publish(output_traj);

  // 8. Publish debug data
  publishDebugMarker();
  publishDebugInfo();

  // 9. Publish and print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  publishCalculationTime(calculation_time);
  RCLCPP_INFO_EXPRESSION(
    get_logger(), is_showing_debug_info_, "%s := %f [ms]", __func__, calculation_time);
}

bool ObstacleCruisePlannerNode::isStopObstacle(const uint8_t label) const
{
  const auto & types = stop_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isCruiseObstacle(const uint8_t label) const
{
  const auto & types = cruise_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isSlowDownObstacle(const uint8_t label) const
{
  const auto & types = slow_down_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

PlannerData ObstacleCruisePlannerNode::createPlannerData(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  PlannerData planner_data;
  planner_data.current_time = now();
  planner_data.traj_points = traj_points;
  planner_data.ego_pose = ego_odom_ptr_->pose.pose;
  planner_data.ego_vel = ego_odom_ptr_->twist.twist.linear.x;
  planner_data.ego_acc = ego_accel_ptr_->accel.accel.linear.x;
  planner_data.is_driving_forward = is_driving_forward_;

  return planner_data;
}

bool ObstacleCruisePlannerNode::isFrontCollideObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
  const size_t first_collision_idx)
{
  const auto obstacle_idx = motion_utils::findNearestIndex(traj_points, obstacle.pose.position);

  const double obstacle_to_col_points_distance =
    motion_utils::calcSignedArcLength(traj_points, obstacle_idx, first_collision_idx);
  const double obstacle_max_length = calcObstacleMaxLength(obstacle.shape);

  // If the obstacle is far in front of the collision point, the obstacle is behind the ego.
  return obstacle_to_col_points_distance > -obstacle_max_length;
}

std::vector<Obstacle> ObstacleCruisePlannerNode::convertToObstacles(
  const std::vector<TrajectoryPoint> & traj_points)
{
  stop_watch_.tic(__func__);

  const auto stamp = rclcpp::Time(objects_ptr_->header.stamp);

  std::vector<Obstacle> target_obstacles;
  for (const auto & predicted_object : objects_ptr_->objects) {
    const auto & object_id =
      tier4_autoware_utils::toHexString(predicted_object.object_id).substr(0, 4);

    // 1. Check if the obstacle's label is target
    const uint8_t label = predicted_object.classification.front().label;
    const bool is_target_obstacle =
      isStopObstacle(label) || isCruiseObstacle(label) || isSlowDownObstacle(label);
    if (!is_target_obstacle) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_, "Ignore obstacle (%s) since its label is not target.",
        object_id.c_str());
      continue;
    }

    // 2. Check if the obstacle is in front of the ego.
    const auto & ego_pose = ego_odom_ptr_->pose.pose;
    const size_t ego_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, ego_pose, ego_nearest_param_.dist_threshold, ego_nearest_param_.yaw_threshold);
    const auto & current_obstacle_pose = obstacle_cruise_utils::getCurrentObjectPose(
      predicted_object, rclcpp::Time(objects_ptr_->header.stamp), now(), true);
    const bool is_front_obstacle =
      isFrontObstacle(traj_points, ego_idx, current_obstacle_pose.pose.position);
    if (!is_front_obstacle) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_,
        "Ignore obstacle (%s) since it is not front obstacle.", object_id.c_str());
      continue;
    }

    // 3. Check if rough lateral distance is smaller than the threshold
    const double lat_dist_from_obstacle_to_traj =
      motion_utils::calcLateralOffset(traj_points, current_obstacle_pose.pose.position);
    const double obstacle_max_length = calcObstacleMaxLength(predicted_object.shape);
    const double rough_lat_dist_to_traj = std::abs(lat_dist_from_obstacle_to_traj) -
                                          vehicle_info_.vehicle_width_m - obstacle_max_length;
    if (obstacle_filtering_param_.rough_detection_area_expand_width < rough_lat_dist_to_traj) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_,
        "Ignore obstacle (%s) since it is far from the trajectory.", object_id.c_str());
      continue;
    }

    const auto target_obstacle =
      Obstacle(stamp, predicted_object, current_obstacle_pose.pose, rough_lat_dist_to_traj);
    target_obstacles.push_back(target_obstacle);
  }

  // Print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), is_showing_debug_info_, "  %s := %f [ms]",
    __func__, calculation_time);

  return target_obstacles;
}

std::tuple<std::vector<StopObstacle>, std::vector<CruiseObstacle>, std::vector<SlowDownObstacle>>
ObstacleCruisePlannerNode::determineEgoBehaviorAgainstObstacles(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Obstacle> & obstacles)
{
  const auto & ego_pose = ego_odom_ptr_->pose.pose;
  const size_t ego_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    traj_points, ego_pose, ego_nearest_param_.dist_threshold, ego_nearest_param_.yaw_threshold);

  const auto extended_traj_points = resampleExtendedTrajectory(traj_points, ego_seg_idx);
  if (!extended_traj_points) {
    // TODO(murooka)
  }

  const auto extended_traj_polys =
    polygon_utils::createOneStepPolygons(*extended_traj_points, vehicle_info_);
  debug_data_ptr_->detection_polygons = extended_traj_polys;

  std::vector<StopObstacle> stop_obstacles;
  std::vector<CruiseObstacle> cruise_obstacles;
  std::vector<SlowDownObstacle> slow_down_obstacles;

  for (const auto & obstacle : obstacles) {
    const auto obstacle_poly = obstacle.toPolygon();

    // Calculate distance between trajectory and obstacle first
    double precise_lat_dist = std::numeric_limits<double>::max();
    for (const auto & traj_poly : extended_traj_polys) {
      const double current_precise_lat_dist = bg::distance(traj_poly, obstacle_poly);
      precise_lat_dist = std::min(precise_lat_dist, current_precise_lat_dist);
    }

    // Filter obstacles for stop, cruise and slow down
    const auto stop_obstacle =
      createStopObstacle(*extended_traj_points, extended_traj_polys, obstacle, precise_lat_dist);
    if (stop_obstacle) {
      stop_obstacles.push_back(*stop_obstacle);
      continue;
    }
    const auto cruise_obstacle =
      createCruiseObstacle(*extended_traj_points, extended_traj_polys, obstacle);
    if (cruise_obstacle) {
      cruise_obstacles.push_back(*cruise_obstacle);
      continue;
    }
    const auto slow_down_obstacle = createSlowDownObstacle(obstacle, precise_lat_dist);
    if (slow_down_obstacle) {
      slow_down_obstacles.push_back(*slow_down_obstacle);
      continue;
    }
  }

  // update previous obstacles
  prev_stop_obstacles_ = stop_obstacles;
  prev_cruise_obstacles_ = cruise_obstacles;
  prev_slow_down_obstacles_ = slow_down_obstacles;

  return {stop_obstacles, cruise_obstacles, slow_down_obstacles};
}

std::optional<std::vector<TrajectoryPoint>> ObstacleCruisePlannerNode::resampleExtendedTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_seg_idx) const
{
  const size_t traj_start_point_idx = ego_seg_idx;

  const auto trimmed_traj_points =
    std::vector<TrajectoryPoint>(traj_points.begin() + traj_start_point_idx, traj_points.end());
  if (trimmed_traj_points.size() < 2) {
    return std::nullopt;
  }

  const auto decimated_traj_points = [&]() {
    const auto decimated_traj = motion_utils::resampleTrajectory(
      motion_utils::convertToTrajectory(trimmed_traj_points),
      obstacle_filtering_param_.decimate_trajectory_step_length);
    return motion_utils::convertToTrajectoryPointArray(decimated_traj);
  }();
  if (decimated_traj_points.size() < 2) {
    return std::nullopt;
  }

  const auto extended_traj_points = extendTrajectoryPoints(
    decimated_traj_points, obstacle_filtering_param_.goal_extension_length,
    obstacle_filtering_param_.goal_extension_interval);

  return extended_traj_points;
}

std::optional<StopObstacle> ObstacleCruisePlannerNode::createStopObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle, [[maybe_unused]] const double precise_lat_dist)
{
  // consider all target obstacles when driving backward
  // rough lateral deviation check
  if (
    disable_stop_planning_ || !isStopObstacle(obstacle.classification.label)
    // ||stop_max_lateral_margin_ < precise_lat_dist) { // TODO(murooka)
  ) {
    return std::nullopt;
  }

  // when driving backward, always do stop planning
  if (is_driving_forward_) {
    // TODO(murooka) use velocity instead of twist.linear.x
    // keep stop obstacles
    const auto prev_stop_obstacle = getObstacleFromId(prev_stop_obstacles_, obstacle.uuid);
    if (prev_stop_obstacle) {
      if (
        obstacle_velocity_threshold_from_stop_to_cruise_ <
        obstacle.twist.linear.x) {  // consider histerysis
        return std::nullopt;
      }
    }

    // stop obstacles from cruise obstacles
    const auto prev_cruise_obstacle = getObstacleFromId(prev_cruise_obstacles_, obstacle.uuid);
    if (prev_cruise_obstacle) {
      if (
        obstacle.twist.linear.x <
        obstacle_velocity_threshold_from_cruise_to_stop_) {  // consider histerysis
        return std::nullopt;
      }
    }
  }

  // precise lateral deviation check
  const auto collision_point = polygon_utils::getCollisionPoint(
    traj_points, traj_polys, obstacle, vehicle_info_.max_longitudinal_offset_m,
    is_driving_forward_);
  if (!collision_point) {
    return std::nullopt;
  }

  const double obstacle_projected_vel = calcObstacleProjectedVelocity(traj_points, obstacle);
  return StopObstacle{obstacle.uuid, obstacle.pose, obstacle_projected_vel, *collision_point};
}

std::optional<CruiseObstacle> ObstacleCruisePlannerNode::createCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle)
{
  if (!isCruiseObstacle(obstacle.classification.label) || is_driving_forward_) {
    return std::nullopt;
  }

  // if (cruise_max_lateral_margin_ < precise_lat_dist) {
  //   return std::nullopt;
  // }

  // Get highest confidence predicted path
  const auto predicted_path = getHighestConfidencePredictedPath(obstacle.predicted_paths);
  const auto resampled_predicted_path = perception_utils::resamplePredictedPath(
    predicted_path, obstacle_filtering_param_.prediction_resampling_time_interval,
    obstacle_filtering_param_.prediction_resampling_time_horizon);

  // precise detection area filtering with polygons
  const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
    // calculate collision
    const auto collision_info = polygon_utils::getCollisionIndex(
      traj_points, traj_polys, obstacle.pose, obstacle.stamp, obstacle.shape);

    if (collision_info) {
      // obstacles inside the trajectory
      return filterInsideCruiseObstacle(
        traj_points, traj_polys, obstacle, resampled_predicted_path);
    }
    // obstacles outside the trajectory
    return filterOutsideCruiseObstacle(traj_points, traj_polys, obstacle, resampled_predicted_path);
  }();
  if (!collision_points) {
    return std::nullopt;
  }

  // For debug
  for (const auto & cp : *collision_points) {
    debug_data_ptr_->collision_points.push_back(cp.point);
  }

  // TODO(murooka)
  // Check target obstacles' consistency
  // checkConsistency(stamp, predicted_objects, traj, target_obstacles);

  const double obstacle_projected_vel = calcObstacleProjectedVelocity(traj_points, obstacle);
  return CruiseObstacle{obstacle.uuid, obstacle.pose, obstacle_projected_vel, *collision_points};
}

std::optional<std::vector<PointWithStamp>> ObstacleCruisePlannerNode::filterInsideCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle, const PredictedPath & resampled_predicted_path)
{
  // calculate nearest collision point
  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::getCollisionPoints(
    traj_points, traj_polys, rclcpp::Time(obstacle.stamp), resampled_predicted_path, obstacle.shape,
    now(), vehicle_info_.max_longitudinal_offset_m, is_driving_forward_, collision_index);
  if (collision_points.empty()) {
    return std::nullopt;
  }

  // ignore running obstacles crossing the ego's trajectory with high speed
  const bool is_obstacle_crossing = isObstacleCrossing(traj_points, obstacle);
  if (!is_obstacle_crossing) {
    return std::nullopt;
  }

  // ignore obstacles that will not collide with ego in a certain time.
  const double collision_time_margin =
    calcCollisionTimeMargin(collision_points, traj_points, is_driving_forward_);
  if (obstacle_filtering_param_.collision_time_margin < collision_time_margin) {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    RCLCPP_INFO_EXPRESSION(
      get_logger(), is_showing_debug_info_,
      "Ignore inside obstacle (%s) since it will not collide with the ego.", object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  return collision_points;
}

std::optional<std::vector<PointWithStamp>> ObstacleCruisePlannerNode::filterOutsideCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle, const PredictedPath & resampled_predicted_path)
{
  const auto & object_id = obstacle.uuid.substr(0, 4);

  const auto & types = obstacle_filtering_param_.ignored_outside_obstacle_types;
  if (std::find(types.begin(), types.end(), obstacle.classification.label) != types.end()) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), is_showing_debug_info_,
      "Ignore outside obstacle (%s) since its type is not designated.", object_id.c_str());
    return std::nullopt;
  }

  // rough lateral deviation check
  const double lat_dist_from_obstacle_to_traj =
    motion_utils::calcLateralOffset(traj_points, obstacle.pose.position);
  const double obstacle_max_length = calcObstacleMaxLength(obstacle.shape);
  if (
    std::abs(lat_dist_from_obstacle_to_traj) >
    vehicle_info_.vehicle_width_m + obstacle_max_length +
      obstacle_filtering_param_.outside_rough_detection_area_expand_width) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), is_showing_debug_info_,
      "Ignore outside obstacle (%s) since it is far from the trajectory.", object_id.c_str());
    return std::nullopt;
  }

  if (
    std::abs(obstacle.twist.linear.x) <
    obstacle_filtering_param_.outside_obstacle_min_velocity_threshold) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), is_showing_debug_info_,
      "Ignore outside obstacle (%s) since the obstacle velocity is low.", object_id.c_str());
    return std::nullopt;
  }

  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::willCollideWithSurroundObstacle(
    traj_points, traj_polys, obstacle.stamp, resampled_predicted_path,  // TODO(murooka)
    obstacle.shape, now(),
    vehicle_info_.vehicle_width_m + obstacle_filtering_param_.rough_detection_area_expand_width,
    obstacle_filtering_param_.ego_obstacle_overlap_time_threshold,
    obstacle_filtering_param_.max_prediction_time_for_collision_check, collision_index,
    vehicle_info_.max_longitudinal_offset_m, is_driving_forward_);

  if (collision_points.empty()) {
    // Ignore vehicle obstacles outside the trajectory, whose predicted path
    // overlaps the ego trajectory in a certain time.
    RCLCPP_INFO_EXPRESSION(
      get_logger(), is_showing_debug_info_,
      "Ignore outside obstacle (%s) since it will not collide with the ego.", object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  // Ignore obstacles behind the ego vehicle.
  // Note: Only using isFrontObstacle(), behind obstacles cannot be filtered
  // properly when the trajectory is crossing or overlapping.
  const size_t first_collision_index = collision_index.front();
  if (!isFrontCollideObstacle(traj_points, obstacle, first_collision_index)) {
    return std::nullopt;
  }
  return collision_points;
}

std::optional<SlowDownObstacle> ObstacleCruisePlannerNode::createSlowDownObstacle(
  [[maybe_unused]] const Obstacle & obstacle, [[maybe_unused]] const double precise_lat_dist)
{
  // if (!isSlowDownObstacle(obstacle.label) || slow_down_max_lateral_margin_ < precise_lat_dist) {
  //   return std::nullopt;
  // }
  //
  // // TODO(murooka)
  // const auto collision_point = getCollisionPoint(traj_points, traj_polygons, obstacle.pose,
  // obstacle.shape, is_driving_forward_); if (!collision_point) {
  //   return std::nullopt;
  // }
  //
  // return SlowDownObstacle{collision_point, precise_lat_dist};
  return std::nullopt;
}

/*
void ObstacleCruisePlannerNode::checkConsistency(
  const rclcpp::Time & current_time, const PredictedObjects & predicted_objects,
  const Trajectory & traj, std::vector<Obstacle> & target_obstacles)
{
  const auto current_closest_obstacle =
    obstacle_cruise_utils::getClosestStopObstacle(traj, target_obstacles);

  // If previous closest obstacle ptr is not set
  if (!prev_closest_obstacle_ptr_) {
    if (current_closest_obstacle) {
      prev_closest_obstacle_ptr_ = std::make_shared<Obstacle>(*current_closest_obstacle);
    }
    return;
  }

  // Put previous closest target obstacle if necessary
  const auto predicted_object_itr = std::find_if(
    predicted_objects.objects.begin(), predicted_objects.objects.end(),
    [&](PredictedObject predicted_object) {
      return tier4_autoware_utils::toHexString(predicted_object.object_id) ==
             prev_closest_obstacle_ptr_->uuid;
    });

  // If previous closest obstacle is not in the current perception lists
  // just return the current target obstacles
  if (predicted_object_itr == predicted_objects.objects.end()) {
    return;
  }

  // Previous closest obstacle is in the perception lists
  const auto target_obstacle_itr = std::find_if(
    target_obstacles.begin(), target_obstacles.end(), [&](const Obstacle target_obstacle) {
      return target_obstacle.uuid == prev_closest_obstacle_ptr_->uuid;
    });

  // Previous closest obstacle is both in the perception lists and target obstacles
  if (target_obstacle_itr != target_obstacles.end()) {
    if (current_closest_obstacle) {
      if ((current_closest_obstacle->uuid == prev_closest_obstacle_ptr_->uuid)) {
        // prev_closest_obstacle is current_closest_obstacle just return the target obstacles(in
        // target obstacles)
        prev_closest_obstacle_ptr_ = std::make_shared<Obstacle>(*current_closest_obstacle);
      } else {
        // New obstacle becomes new stop obstacle
        prev_closest_obstacle_ptr_ = std::make_shared<Obstacle>(*current_closest_obstacle);
      }
    } else {
      // Previous closest stop obstacle becomes cruise obstacle
      prev_closest_obstacle_ptr_ = nullptr;
    }
  } else {
    // prev obstacle is not in the target obstacles, but in the perception list
    const double elapsed_time = (current_time - prev_closest_obstacle_ptr_->stamp).seconds();
    if (
      predicted_object_itr->kinematics.initial_twist_with_covariance.twist.linear.x <
        obstacle_velocity_threshold_from_stop_to_cruise_ &&
      elapsed_time < obstacle_filtering_param_.stop_obstacle_hold_time_threshold) {
      target_obstacles.push_back(*prev_closest_obstacle_ptr_);
      return;
    }

    if (current_closest_obstacle) {
      prev_closest_obstacle_ptr_ = std::make_shared<Obstacle>(*current_closest_obstacle);
    } else {
      prev_closest_obstacle_ptr_ = nullptr;
    }
  }
}
*/

bool ObstacleCruisePlannerNode::isObstacleCrossing(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle) const
{
  const bool is_obstacle_crossing_trajectory = !isAngleAlignedWithTrajectory(
    traj_points, obstacle.pose, obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold);
  if (!is_obstacle_crossing_trajectory) {
    return false;
  }

  const double has_high_speed = obstacle_filtering_param_.crossing_obstacle_velocity_threshold <
                                std::abs(obstacle.twist.linear.x);
  if (!has_high_speed) {
    return false;
  }

  // Only obstacles crossing the ego's trajectory with high speed are considered.
  return true;
}

double ObstacleCruisePlannerNode::calcCollisionTimeMargin(
  const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & traj_points, const bool is_driving_forward) const
{
  const auto & ego_pose = ego_odom_ptr_->pose.pose;
  const double ego_vel = ego_odom_ptr_->twist.twist.linear.x;

  const double time_to_collision = [&]() {
    const double abs_ego_offset = is_driving_forward
                                    ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                    : std::abs(vehicle_info_.min_longitudinal_offset_m);
    const double dist_from_ego_to_obstacle =
      motion_utils::calcSignedArcLength(
        traj_points, ego_pose.position, collision_points.front().point) -
      abs_ego_offset;
    return dist_from_ego_to_obstacle / std::max(1e-6, std::abs(ego_vel));
  }();

  const double time_to_start_cross =
    (rclcpp::Time(collision_points.front().stamp) - now()).seconds();
  const double time_to_end_cross = (rclcpp::Time(collision_points.back().stamp) - now()).seconds();

  if (time_to_collision < time_to_start_cross) {  // Ego goes first.
    return time_to_start_cross;
  }
  if (time_to_end_cross < time_to_collision) {  // Obstacle goes first.
    return time_to_collision - time_to_end_cross;
  }
  return 0.0;
}

void ObstacleCruisePlannerNode::publishVelocityLimit(
  const std::optional<VelocityLimit> & vel_limit, const std::string & module_name)
{
  if (vel_limit) {
    vel_limit_pub_->publish(*vel_limit);
    need_to_clear_vel_limit_.at(module_name) = true;
    return;
  }

  if (!need_to_clear_vel_limit_.at(module_name)) {
    return;
  }

  // clear velocity limit
  const auto clear_vel_limit_msg = createVelocityLimitClearCommandMessage(now(), module_name);
  clear_vel_limit_pub_->publish(clear_vel_limit_msg);
  need_to_clear_vel_limit_.at(module_name) = false;
}

void ObstacleCruisePlannerNode::publishDebugMarker() const
{
  stop_watch_.tic(__func__);

  // 1. publish debug marker
  MarkerArray debug_marker;

  // obstacles to cruise
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_cruise.size(); ++i) {
    const auto marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_cruise.at(i).pose, i, "obstacles_to_cruise", 0.7, 0.7, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // obstacles to stop
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_stop.size(); ++i) {
    const auto marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_stop.at(i).pose, i, "obstacles_to_stop", 1.0, 0.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // intentionally ignored obstacles to cruise or stop
  for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->intentionally_ignored_obstacles.at(i).pose, i,
      "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  {  // footprint polygons
    auto marker = tier4_autoware_utils::createDefaultMarker(
      "map", now(), "detection_polygons", 0, Marker::LINE_LIST,
      tier4_autoware_utils::createMarkerScale(0.01, 0.0, 0.0),
      tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (const auto & detection_polygon : debug_data_ptr_->detection_polygons) {
      for (size_t dp_idx = 0; dp_idx < detection_polygon.outer().size(); ++dp_idx) {
        const auto & current_point = detection_polygon.outer().at(dp_idx);
        const auto & next_point =
          detection_polygon.outer().at((dp_idx + 1) % detection_polygon.outer().size());

        marker.points.push_back(
          tier4_autoware_utils::createPoint(current_point.x(), current_point.y(), 0.0));
        marker.points.push_back(
          tier4_autoware_utils::createPoint(next_point.x(), next_point.y(), 0.0));
      }
    }
    debug_marker.markers.push_back(marker);
  }

  {  // collision points
    for (size_t i = 0; i < debug_data_ptr_->collision_points.size(); ++i) {
      auto marker = tier4_autoware_utils::createDefaultMarker(
        "map", now(), "collision_points", i, Marker::SPHERE,
        tier4_autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
        tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      marker.pose.position = debug_data_ptr_->collision_points.at(i);
      debug_marker.markers.push_back(marker);
    }
  }

  debug_marker_pub_->publish(debug_marker);

  // 2. publish virtual wall for cruise and stop
  debug_cruise_wall_marker_pub_->publish(debug_data_ptr_->cruise_wall_marker);
  debug_stop_wall_marker_pub_->publish(debug_data_ptr_->stop_wall_marker);

  // 3. print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), is_showing_debug_info_, "  %s := %f [ms]",
    __func__, calculation_time);
}

void ObstacleCruisePlannerNode::publishDebugInfo() const
{
  // stop
  const auto stop_debug_msg = planner_ptr_->getStopPlanningDebugMessage(now());
  debug_stop_planning_info_pub_->publish(stop_debug_msg);

  // cruise
  const auto cruise_debug_msg = planner_ptr_->getCruisePlanningDebugMessage(now());
  debug_cruise_planning_info_pub_->publish(cruise_debug_msg);

  // slow_down
  // const auto slow_down_debug_msg = planner_ptr_->getSlowDownPlanningDebugMessage(now());
  // debug_slow_down_planning_info_pub_->publish(slow_down_debug_msg);
}

void ObstacleCruisePlannerNode::publishCalculationTime(const double calculation_time) const
{
  Float32Stamped calculation_time_msg;
  calculation_time_msg.stamp = now();
  calculation_time_msg.data = calculation_time;
  debug_calculation_time_pub_->publish(calculation_time_msg);
}
}  // namespace motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleCruisePlannerNode)
