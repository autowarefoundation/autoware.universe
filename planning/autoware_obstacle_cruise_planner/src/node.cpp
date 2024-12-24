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

#include "autoware/obstacle_cruise_planner/node.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_cruise_initializer.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
std::optional<double> calcDistanceToFrontVehicle(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos)
{
  const size_t obstacle_idx = autoware::motion_utils::findNearestIndex(traj_points, obstacle_pos);
  const auto ego_to_obstacle_distance =
    autoware::motion_utils::calcSignedArcLength(traj_points, ego_idx, obstacle_idx);
  if (ego_to_obstacle_distance < 0.0) return std::nullopt;
  return ego_to_obstacle_distance;
}

/**
 * @brief Calculates the obstacle's longitudinal and approach velocities relative to the trajectory.
 *
 * This function calculates the obstacle's velocity components relative to the trajectory.
 * It returns the longitudinal and approach components of the obstacle's velocity
 * with respect to the trajectory. Negative approach velocity indicates that the
 * obstacle is getting far away from the trajectory.
 *
 * @param traj_points The trajectory points.
 * @param obstacle_pose The current pose of the obstacle.
 * @param obstacle_twist The twist (velocity) of the obstacle.
 * @return A pair containing the longitudinal and approach velocity components.
 */
std::pair<double, double> calculateObstacleVelocitiesRelativeToTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & obstacle_pose,
  const geometry_msgs::msg::Twist & obstacle_twist)
{
  const size_t object_idx =
    autoware::motion_utils::findNearestIndex(traj_points, obstacle_pose.position);

  const auto & nearest_point = traj_points.at(object_idx);

  const double traj_yaw = tf2::getYaw(nearest_point.pose.orientation);
  const double obstacle_yaw = tf2::getYaw(obstacle_pose.orientation);
  const Eigen::Rotation2Dd R_ego_to_obstacle(
    autoware::universe_utils::normalizeRadian(obstacle_yaw - traj_yaw));

  // Calculate the trajectory direction and the vector from the trajectory to the obstacle
  const Eigen::Vector2d traj_direction(std::cos(traj_yaw), std::sin(traj_yaw));
  const Eigen::Vector2d traj_to_obstacle(
    obstacle_pose.position.x - nearest_point.pose.position.x,
    obstacle_pose.position.y - nearest_point.pose.position.y);

  // Determine if the obstacle is to the left or right of the trajectory using the cross product
  const double cross_product =
    traj_direction.x() * traj_to_obstacle.y() - traj_direction.y() * traj_to_obstacle.x();
  const int sign = (cross_product > 0) ? -1 : 1;

  const Eigen::Vector2d obstacle_velocity(obstacle_twist.linear.x, obstacle_twist.linear.y);
  const Eigen::Vector2d projected_velocity = R_ego_to_obstacle * obstacle_velocity;

  return std::make_pair(projected_velocity[0], sign * projected_velocity[1]);
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point, const bool is_driving_forward)
{
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = autoware::universe_utils::calcOffsetPose(
    goal_point.pose, extend_distance * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
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
  const auto is_driving_forward_opt =
    autoware::motion_utils::isDrivingForwardWithTwist(input_points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output_points;
  }

  const auto goal_point = input_points.back();

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - step_length)) {
    const auto extend_trajectory_point =
      getExtendTrajectoryPoint(extend_sum, goal_point, is_driving_forward);
    output_points.push_back(extend_trajectory_point);
    extend_sum += step_length;
  }
  const auto extend_trajectory_point =
    getExtendTrajectoryPoint(extend_distance, goal_point, is_driving_forward);
  output_points.push_back(extend_trajectory_point);

  return output_points;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints2(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

geometry_msgs::msg::Point toGeomPoint2(const pcl::PointXYZ & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x;
  geom_point.y = point.y;
  geom_point.z = point.z;
  return geom_point;
}

template <typename T>
void concatenate(std::vector<T> & first, const std::vector<T> & last)
{
  first.insert(first.end(), last.begin(), last.end());
}
}  // namespace

namespace autoware::motion_planning
{
ObstacleCruisePlannerNode::ObstacleCruisePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_cruise_planner", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  debug_data_ptr_(std::make_shared<DebugData>())
{
  using std::placeholders::_1;

  // subscriber
  traj_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleCruisePlannerNode::onTrajectory, this, _1));

  // publisher
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);

  // debug publisher
  debug_calculation_time_pub_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);
  debug_marker_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);

  // tf listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto longitudinal_info = LongitudinalInfo(*this);

  ego_nearest_param_ = EgoNearestParam(*this);

  enable_debug_info_ = declare_parameter<bool>("common.enable_debug_info");
  enable_calculation_time_info_ = declare_parameter<bool>("common.enable_calculation_time_info");

  // use_pointcloud_ = use_pointcloud_for_stop_ || use_pointcloud_for_slow_down_;

  behavior_determination_param_ = BehaviorDeterminationParam(*this);

  {  // planning algorithm
    obstacle_stop_module_ = std::make_unique<ObstacleStopModule>(
      *this, longitudinal_info, vehicle_info_, ego_nearest_param_);
    obstacle_slow_down_module_ = std::make_unique<ObstacleSlowDownModule>(
      *this, longitudinal_info, vehicle_info_, ego_nearest_param_);
    obstacle_cruise_module_ =
      getModule(*this, longitudinal_info, vehicle_info_, ego_nearest_param_);

    min_behavior_stop_margin_ = declare_parameter<double>("common.min_behavior_stop_margin");
    additional_safe_distance_margin_on_curve_ =
      declare_parameter<double>("common.stop_on_curve.additional_safe_distance_margin");
    enable_approaching_on_curve_ =
      declare_parameter<bool>("common.stop_on_curve.enable_approaching");
    min_safe_distance_margin_on_curve_ =
      declare_parameter<double>("common.stop_on_curve.min_safe_distance_margin");
    suppress_sudden_obstacle_stop_ =
      declare_parameter<bool>("common.suppress_sudden_obstacle_stop");
    obstacle_stop_module_->setParam(
      enable_debug_info_, enable_calculation_time_info_, true /*use_pointcloud_*/,
      min_behavior_stop_margin_, enable_approaching_on_curve_,
      additional_safe_distance_margin_on_curve_, min_safe_distance_margin_on_curve_,
      suppress_sudden_obstacle_stop_);
    obstacle_slow_down_module_->setParam(
      enable_debug_info_, enable_calculation_time_info_, true /*use_pointcloud_*/,
      min_behavior_stop_margin_, enable_approaching_on_curve_,
      additional_safe_distance_margin_on_curve_, min_safe_distance_margin_on_curve_,
      suppress_sudden_obstacle_stop_);
    obstacle_cruise_module_->setParam(
      enable_debug_info_, enable_calculation_time_info_, true /*use_pointcloud_*/,
      min_behavior_stop_margin_, enable_approaching_on_curve_,
      additional_safe_distance_margin_on_curve_, min_safe_distance_margin_on_curve_,
      suppress_sudden_obstacle_stop_);
  }

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleCruisePlannerNode::onParam, this, std::placeholders::_1));

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}

rcl_interfaces::msg::SetParametersResult ObstacleCruisePlannerNode::onParam(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  //   planner_ptr_->onParam(parameters);
  //
  //   autoware::universe_utils::updateParam<bool>(
  //     parameters, "common.enable_debug_info", enable_debug_info_);
  //   autoware::universe_utils::updateParam<bool>(
  //     parameters, "common.enable_calculation_time_info", enable_calculation_time_info_);
  //
  //   autoware::universe_utils::updateParam<bool>(
  //     parameters, "common.stop_on_curve.enable_approaching", enable_approaching_on_curve_);
  //   autoware::universe_utils::updateParam<double>(
  //     parameters, "common.stop_on_curve.additional_safe_distance_margin",
  //     additional_safe_distance_margin_on_curve_);
  //   autoware::universe_utils::updateParam<double>(
  //     parameters, "common.stop_on_curve.min_safe_distance_margin",
  //     min_safe_distance_margin_on_curve_);
  //
  //   planner_ptr_->setParam(
  //     enable_debug_info_, enable_calculation_time_info_, use_pointcloud_,
  //     min_behavior_stop_margin_, enable_approaching_on_curve_,
  //     additional_safe_distance_margin_on_curve_, min_safe_distance_margin_on_curve_,
  //     suppress_sudden_obstacle_stop_);
  //
  //   autoware::universe_utils::updateParam<bool>(
  //     parameters, "common.enable_slow_down_planning", enable_slow_down_planning_);
  //
  //   behavior_determination_param_.onParam(parameters);
  //
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ObstacleCruisePlannerNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  const auto ego_odom_ptr = ego_odom_sub_.takeData();
  const auto objects_ptr = objects_sub_.takeData();
  const auto pointcloud_ptr = pointcloud_sub_.takeData();
  const auto acc_ptr = acc_sub_.takeData();
  if (!ego_odom_ptr || !acc_ptr) {
    return;
  }

  const auto & ego_odom = *ego_odom_ptr;
  const auto & acc = *acc_ptr;

  const auto traj_points = autoware::motion_utils::convertToTrajectoryPointArray(*msg);
  // check if subscribed variables are ready
  if (traj_points.empty()) {
    return;
  }

  stop_watch_.tic(__func__);
  *debug_data_ptr_ = DebugData();

  const auto is_driving_forward = autoware::motion_utils::isDrivingForwardWithTwist(traj_points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;

  /*
  const auto & [stop_obstacles, cruise_obstacles, slow_down_obstacles] = [&]() {
    std::vector<StopObstacle> stop_obstacles;
    std::vector<CruiseObstacle> cruise_obstacles;
    std::vector<SlowDownObstacle> slow_down_obstacles;
    if (objects_ptr) {
      // 1. Convert predicted objects to obstacles which are
      //    (1) with a proper label
      //    (2) in front of ego
      //    (3) not too far from trajectory
      const auto target_obstacles = convertToObstacles(ego_odom, *objects_ptr, traj_points);

      const auto stop_obstacles =
  obstacle_stop_module_->determineEgoBehaviorAgainstPredictedObjectObstacles(ego_odom, *objects_ptr,
  traj_points, target_obstacles); const auto slow_down_obstacles =
  obstacle_slow_down_module_->determineEgoBehaviorAgainstPredictedObjectObstacles(ego_odom,
  *objects_ptr, traj_points, target_obstacles); const auto cruise_obstacles =
  obstacle_cruise_module_->determineEgoBehaviorAgainstPredictedObjectObstacles(ego_odom,
  *objects_ptr, traj_points, target_obstacles);

      concatenate(stop_obstacles, stop_object_obstacles);
      concatenate(cruise_obstacles, cruise_object_obstacles);
      concatenate(slow_down_obstacles, slow_down_object_obstacles);
    }
    if (use_pointcloud_) {
      const auto target_obstacles =
        convertToObstacles(ego_odom, *pointcloud_ptr, traj_points, msg->header);

      const auto & [stop_pc_obstacles, cruise_pc_obstacles, slow_down_pc_obstacles] =
        determineEgoBehaviorAgainstPointCloudObstacles(ego_odom, traj_points, target_obstacles);

      concatenate(stop_obstacles, stop_pc_obstacles);
      concatenate(cruise_obstacles, cruise_pc_obstacles);
      concatenate(slow_down_obstacles, slow_down_pc_obstacles);
    }
    return std::make_tuple(stop_obstacles, cruise_obstacles, slow_down_obstacles);
  }();
  */

  // 3. Create data for planning
  const auto planner_data = createPlannerData(ego_odom, acc, traj_points);

  // calculated decimated trajectory points and trajectory polygon
  const auto decimated_traj_points = decimateTrajectoryPoints(ego_odom, traj_points);
  const auto decimated_traj_polys = obstacle_cruise_utils::createOneStepPolygons(
    decimated_traj_points, vehicle_info_, ego_odom.pose.pose, 0.0, behavior_determination_param_);
  debug_data_ptr_->detection_polygons = decimated_traj_polys;

  const auto target_obstacles = convertToObstacles(ego_odom, *objects_ptr, traj_points);

  // stop
  const auto stop_obstacles =
    obstacle_stop_module_->determineEgoBehaviorAgainstPredictedObjectObstacles(
      ego_odom, *objects_ptr, decimated_traj_points, decimated_traj_polys, target_obstacles,
      is_driving_forward_, behavior_determination_param_, min_behavior_stop_margin_);
  const auto stop_traj_points =
    obstacle_stop_module_->generateStopTrajectory(planner_data, stop_obstacles);

  // cruise
  const auto cruise_obstacles =
    obstacle_cruise_module_->determineEgoBehaviorAgainstPredictedObjectObstacles(
      ego_odom, decimated_traj_points, decimated_traj_polys, target_obstacles, is_driving_forward_,
      behavior_determination_param_);
  std::optional<VelocityLimit> cruise_vel_limit;
  const auto cruise_traj_points = obstacle_cruise_module_->generateCruiseTrajectory(
    planner_data, stop_traj_points, cruise_obstacles, cruise_vel_limit);
  obstacle_cruise_module_->publishVelocityLimit(cruise_vel_limit);

  // slow down
  const auto slow_down_obstacles =
    obstacle_slow_down_module_->determineEgoBehaviorAgainstPredictedObjectObstacles(
      ego_odom, decimated_traj_points, target_obstacles, behavior_determination_param_);
  std::optional<VelocityLimit> slow_down_vel_limit;
  const auto slow_down_traj_points = obstacle_slow_down_module_->generateSlowDownTrajectory(
    planner_data, cruise_traj_points, slow_down_obstacles, slow_down_vel_limit);
  obstacle_slow_down_module_->publishVelocityLimit(slow_down_vel_limit);

  // 7. Publish trajectory
  const auto output_traj =
    autoware::motion_utils::convertToTrajectory(slow_down_traj_points, msg->header);
  trajectory_pub_->publish(output_traj);

  // 8. Publish debug data
  published_time_publisher_->publish_if_subscribed(trajectory_pub_, output_traj.header.stamp);

  obstacle_stop_module_->publishMetrics(now());
  obstacle_slow_down_module_->publishMetrics(now());
  obstacle_cruise_module_->publishMetrics(now());

  obstacle_stop_module_->postprocess();
  obstacle_slow_down_module_->postprocess();
  obstacle_cruise_module_->postprocess();

  publishDebugMarker();

  // 9. Publish and print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  publishCalculationTime(calculation_time);
  RCLCPP_INFO_EXPRESSION(
    get_logger(), enable_calculation_time_info_, "%s := %f [ms]", __func__, calculation_time);
}

std::vector<Obstacle> ObstacleCruisePlannerNode::convertToObstacles(
  const Odometry & odometry, const PredictedObjects & objects,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  stop_watch_.tic(__func__);

  const auto obj_stamp = rclcpp::Time(objects.header.stamp);
  const auto & p = behavior_determination_param_;

  constexpr double epsilon = 1e-6;
  const double max_lat_margin = std::max(
    {p.max_lat_margin_for_stop, p.max_lat_margin_for_stop_against_unknown,
     p.max_lat_margin_for_cruise, p.max_lat_margin_for_slow_down});
  const double max_lat_time_margin =
    std::max({p.max_lat_time_margin_for_stop, p.max_lat_time_margin_for_cruise});
  const size_t ego_idx = ego_nearest_param_.findIndex(traj_points, odometry.pose.pose);

  std::vector<Obstacle> target_obstacles;
  for (const auto & predicted_object : objects.objects) {
    const auto & object_id =
      autoware::universe_utils::toHexString(predicted_object.object_id).substr(0, 4);

    // brkay54: When use_prediction is true, we observed wrong orientation for the object in
    // scenario simulator.
    const auto & current_obstacle_pose =
      obstacle_cruise_utils::getCurrentObjectPose(predicted_object, obj_stamp, now(), true);

    // 1. Check if the obstacle's label is target
    /*
    const uint8_t label = predicted_object.classification.front().label;
    const bool is_target_obstacle =
      isStopObstacle(label) || isCruiseObstacle(label) || isSlowDownObstacle(label);
    if (!is_target_obstacle) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_, "Ignore obstacle (%s) since its label is not target.",
        object_id.c_str());
      continue;
    }
    */

    const auto projected_vel = calculateObstacleVelocitiesRelativeToTrajectory(
      traj_points, current_obstacle_pose.pose,
      predicted_object.kinematics.initial_twist_with_covariance.twist);

    // 2. Check if the obstacle is in front of the ego.
    const auto ego_to_obstacle_distance =
      calcDistanceToFrontVehicle(traj_points, ego_idx, current_obstacle_pose.pose.position);
    if (!ego_to_obstacle_distance) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_, "Ignore obstacle (%s) since it is not front obstacle.",
        object_id.c_str());
      continue;
    }

    // 3. Check if rough lateral distance and time to reach trajectory are smaller than the
    // threshold
    const double lat_dist_from_obstacle_to_traj =
      autoware::motion_utils::calcLateralOffset(traj_points, current_obstacle_pose.pose.position);

    const double min_lat_dist_to_traj_poly = [&]() {
      const double obstacle_max_length =
        obstacle_cruise_utils::calcObstacleMaxLength(predicted_object.shape);
      return std::abs(lat_dist_from_obstacle_to_traj) - vehicle_info_.vehicle_width_m -
             obstacle_max_length;
    }();

    if (max_lat_margin < min_lat_dist_to_traj_poly) {
      if (projected_vel.second > 0.0) {
        const auto time_to_traj =
          min_lat_dist_to_traj_poly / std::max(epsilon, projected_vel.second);
        if (time_to_traj > max_lat_time_margin) {
          RCLCPP_INFO_EXPRESSION(
            get_logger(), enable_debug_info_,
            "Ignore obstacle (%s) since it is too far from the trajectory.", object_id.c_str());
          continue;
        }
      } else {
        RCLCPP_INFO_EXPRESSION(
          get_logger(), enable_debug_info_,
          "Ignore obstacle (%s) since it is too far from the trajectory.", object_id.c_str());
        continue;
      }
    }

    const auto decimated_traj_points = decimateTrajectoryPoints(odometry, traj_points);
    const auto decimated_traj_polys = obstacle_cruise_utils::createOneStepPolygons(
      decimated_traj_points, vehicle_info_, odometry.pose.pose, 0.0, behavior_determination_param_);
    debug_data_ptr_->detection_polygons = decimated_traj_polys;

    const auto obstacle_poly =
      autoware::universe_utils::toPolygon2d(current_obstacle_pose.pose, predicted_object.shape);

    // Calculate distance between trajectory and obstacle first
    double precise_lat_dist = std::numeric_limits<double>::max();
    for (const auto & traj_poly : decimated_traj_polys) {
      const double current_precise_lat_dist = bg::distance(traj_poly, obstacle_poly);
      precise_lat_dist = std::min(precise_lat_dist, current_precise_lat_dist);
    }

    const auto target_obstacle = Obstacle(
      obj_stamp, predicted_object, current_obstacle_pose.pose, *ego_to_obstacle_distance,
      lat_dist_from_obstacle_to_traj, projected_vel.first, projected_vel.second, precise_lat_dist);
    target_obstacles.push_back(target_obstacle);
  }

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_debug_info_, "  %s := %f [ms]", __func__,
    calculation_time);

  return target_obstacles;
}

std::vector<Obstacle> ObstacleCruisePlannerNode::convertToObstacles(
  const Odometry & odometry, const PointCloud2 & pointcloud,
  const std::vector<TrajectoryPoint> & traj_points, const std_msgs::msg::Header & traj_header) const
{
  stop_watch_.tic(__func__);

  const auto & p = behavior_determination_param_;

  std::vector<Obstacle> target_obstacles;

  std::optional<geometry_msgs::msg::TransformStamped> transform_stamped{};
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      traj_header.frame_id, pointcloud.header.frame_id, pointcloud.header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to look up transform from " << traj_header.frame_id << " to "
                                                        << pointcloud.header.frame_id);
    transform_stamped = std::nullopt;
  }

  if (!pointcloud.data.empty() && transform_stamped) {
    // 1. transform pointcloud
    PointCloud::Ptr pointcloud_ptr(new PointCloud);
    pcl::fromROSMsg(pointcloud, *pointcloud_ptr);
    const Eigen::Matrix4f transform =
      tf2::transformToEigen(transform_stamped.value().transform).matrix().cast<float>();
    pcl::transformPointCloud(*pointcloud_ptr, *pointcloud_ptr, transform);

    // 2. downsample & cluster pointcloud
    PointCloud::Ptr filtered_points_ptr(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(pointcloud_ptr);
    filter.setLeafSize(
      p.pointcloud_voxel_grid_x, p.pointcloud_voxel_grid_y, p.pointcloud_voxel_grid_z);
    filter.filter(*filtered_points_ptr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered_points_ptr);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(p.pointcloud_cluster_tolerance);
    ec.setMinClusterSize(p.pointcloud_min_cluster_size);
    ec.setMaxClusterSize(p.pointcloud_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_points_ptr);
    ec.extract(clusters);

    const auto max_lat_margin =
      std::max(p.max_lat_margin_for_stop_against_unknown, p.max_lat_margin_for_slow_down);
    const size_t ego_idx = ego_nearest_param_.findIndex(traj_points, odometry.pose.pose);

    // 3. convert clusters to obstacles
    for (const auto & cluster_indices : clusters) {
      double ego_to_stop_collision_distance = std::numeric_limits<double>::max();
      double ego_to_slow_down_front_collision_distance = std::numeric_limits<double>::max();
      double ego_to_slow_down_back_collision_distance = std::numeric_limits<double>::min();
      double lat_dist_from_obstacle_to_traj = std::numeric_limits<double>::max();
      double ego_to_obstacle_distance = std::numeric_limits<double>::max();
      std::optional<geometry_msgs::msg::Point> stop_collision_point = std::nullopt;
      std::optional<geometry_msgs::msg::Point> slow_down_front_collision_point = std::nullopt;
      std::optional<geometry_msgs::msg::Point> slow_down_back_collision_point = std::nullopt;

      for (const auto & index : cluster_indices.indices) {
        const auto obstacle_point = toGeomPoint2(filtered_points_ptr->points[index]);
        const auto current_lat_dist_from_obstacle_to_traj =
          autoware::motion_utils::calcLateralOffset(traj_points, obstacle_point);
        const auto min_lat_dist_to_traj_poly =
          std::abs(current_lat_dist_from_obstacle_to_traj) - vehicle_info_.vehicle_width_m;

        if (min_lat_dist_to_traj_poly < max_lat_margin) {
          const auto current_ego_to_obstacle_distance =
            calcDistanceToFrontVehicle(traj_points, ego_idx, obstacle_point);
          if (current_ego_to_obstacle_distance) {
            ego_to_obstacle_distance =
              std::min(ego_to_obstacle_distance, *current_ego_to_obstacle_distance);
          } else {
            continue;
          }

          lat_dist_from_obstacle_to_traj =
            std::min(lat_dist_from_obstacle_to_traj, current_lat_dist_from_obstacle_to_traj);

          if (min_lat_dist_to_traj_poly < p.max_lat_margin_for_stop_against_unknown) {
            if (*current_ego_to_obstacle_distance < ego_to_stop_collision_distance) {
              stop_collision_point = obstacle_point;
              ego_to_stop_collision_distance = *current_ego_to_obstacle_distance;
            }
          } else if (min_lat_dist_to_traj_poly < p.max_lat_margin_for_slow_down) {
            if (*current_ego_to_obstacle_distance < ego_to_slow_down_front_collision_distance) {
              slow_down_front_collision_point = obstacle_point;
              ego_to_slow_down_front_collision_distance = *current_ego_to_obstacle_distance;
            } else if (
              *current_ego_to_obstacle_distance > ego_to_slow_down_back_collision_distance) {
              slow_down_back_collision_point = obstacle_point;
              ego_to_slow_down_back_collision_distance = *current_ego_to_obstacle_distance;
            }
          }
        }
      }

      if (stop_collision_point || slow_down_front_collision_point) {
        target_obstacles.emplace_back(
          pointcloud.header.stamp, stop_collision_point, slow_down_front_collision_point,
          slow_down_back_collision_point, ego_to_obstacle_distance, lat_dist_from_obstacle_to_traj);
      }
    }
  }

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_debug_info_, "  %s := %f [ms]", __func__,
    calculation_time);

  return target_obstacles;
}

/*
std::tuple<std::vector<StopObstacle>, std::vector<CruiseObstacle>, std::vector<SlowDownObstacle>>
ObstacleCruisePlannerNode::determineEgoBehaviorAgainstPointCloudObstacles(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Obstacle> & obstacles)
{
  const auto & p = behavior_determination_param_;

  // calculated decimated trajectory points and trajectory polygon
  const auto decimated_traj_points = decimateTrajectoryPoints(odometry, traj_points);
  const auto decimated_traj_polys =
    createOneStepPolygons(decimated_traj_points, vehicle_info_, odometry.pose.pose);
  debug_data_ptr_->detection_polygons = decimated_traj_polys;

  // determine ego's behavior from stop and slow down
  std::vector<StopObstacle> stop_obstacles;
  for (const auto & obstacle : obstacles) {
    const auto & precise_lat_dist = obstacle.lat_dist_from_obstacle_to_traj;
    const auto stop_obstacle =
      createStopObstacleForPointCloud(decimated_traj_points, obstacle, precise_lat_dist);
    if (stop_obstacle) {
      stop_obstacles.push_back(*stop_obstacle);
    }
  }

  std::vector<StopObstacle> past_stop_obstacles;
  for (auto itr = stop_pc_obstacle_history_.begin(); itr != stop_pc_obstacle_history_.end();) {
    const double elapsed_time = (rclcpp::Time(odometry.header.stamp) - itr->stamp).seconds();
    if (elapsed_time >= p.stop_obstacle_hold_time_threshold) {
      itr = stop_pc_obstacle_history_.erase(itr);
      continue;
    }

    const auto lat_dist_from_obstacle_to_traj =
      autoware::motion_utils::calcLateralOffset(traj_points, itr->collision_point);
    const auto min_lat_dist_to_traj_poly =
      std::abs(lat_dist_from_obstacle_to_traj) - vehicle_info_.vehicle_width_m;

    if (min_lat_dist_to_traj_poly < p.max_lat_margin_for_stop_against_unknown) {
      auto stop_obstacle = *itr;
      stop_obstacle.dist_to_collide_on_decimated_traj = autoware::motion_utils::calcSignedArcLength(
        decimated_traj_points, 0, stop_obstacle.collision_point);
      past_stop_obstacles.push_back(stop_obstacle);
    }

    ++itr;
  }

  concatenate(stop_pc_obstacle_history_, stop_obstacles);
  concatenate(stop_obstacles, past_stop_obstacles);

  // slow down
  std::vector<SlowDownObstacle> slow_down_obstacles;
  for (const auto & obstacle : obstacles) {
    const auto & precise_lat_dist = obstacle.lat_dist_from_obstacle_to_traj;
    const auto slow_down_obstacle = createSlowDownObstacleForPointCloud(obstacle, precise_lat_dist);
    if (slow_down_obstacle) {
      slow_down_obstacles.push_back(*slow_down_obstacle);
    }
  }

  return {stop_obstacles, {}, slow_down_obstacles};
}
*/

std::vector<TrajectoryPoint> ObstacleCruisePlannerNode::decimateTrajectoryPoints(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto & p = behavior_determination_param_;

  // trim trajectory
  const size_t ego_seg_idx = ego_nearest_param_.findSegmentIndex(traj_points, odometry.pose.pose);
  const size_t traj_start_point_idx = ego_seg_idx;
  const auto trimmed_traj_points =
    std::vector<TrajectoryPoint>(traj_points.begin() + traj_start_point_idx, traj_points.end());

  // decimate trajectory
  const auto decimated_traj_points =
    resampleTrajectoryPoints2(trimmed_traj_points, p.decimate_trajectory_step_length);

  // extend trajectory
  const auto extended_traj_points = extendTrajectoryPoints(
    decimated_traj_points, obstacle_stop_module_->getSafeDistanceMargin(),
    p.decimate_trajectory_step_length);
  if (extended_traj_points.size() < 2) {
    return traj_points;
  }
  return extended_traj_points;
}

/*
std::optional<StopObstacle> ObstacleCruisePlannerNode::createStopObstacleForPointCloud(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
  const double precise_lat_dist) const
{
  const auto & p = behavior_determination_param_;

  if (!use_pointcloud_for_stop_) {
    return std::nullopt;
  }

  if (!obstacle.stop_collision_point) {
    return std::nullopt;
  }

  const double max_lat_margin_for_stop =
    (obstacle.classification.label == ObjectClassification::UNKNOWN)
      ? p.max_lat_margin_for_stop_against_unknown
      : p.max_lat_margin_for_stop;

  if (precise_lat_dist > std::max(max_lat_margin_for_stop, 1e-3)) {
    return std::nullopt;
  }

  const auto dist_to_collide_on_traj =
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, *obstacle.stop_collision_point);

  return StopObstacle{
    obstacle.uuid,
    obstacle.stamp,
    obstacle.classification,
    obstacle.pose,
    obstacle.shape,
    {},
    {},
    *obstacle.stop_collision_point,
    dist_to_collide_on_traj};
}
*/

PlannerData ObstacleCruisePlannerNode::createPlannerData(
  const Odometry & odometry, const AccelWithCovarianceStamped & acc,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  PlannerData planner_data;
  planner_data.current_time = now();
  planner_data.traj_points = traj_points;
  planner_data.ego_pose = odometry.pose.pose;
  planner_data.ego_vel = odometry.twist.twist.linear.x;
  planner_data.ego_acc = acc.accel.accel.linear.x;
  planner_data.is_driving_forward = is_driving_forward_;
  return planner_data;
}

void ObstacleCruisePlannerNode::publishDebugMarker() const
{
  stop_watch_.tic(__func__);
  obstacle_stop_module_->publishDebugMarker();
  obstacle_slow_down_module_->publishDebugMarker();
  obstacle_cruise_module_->publishDebugMarker();

  // {  // footprint polygons
  //   auto marker = autoware::universe_utils::createDefaultMarker(
  //     "map", now(), "detection_polygons", 0, Marker::LINE_LIST,
  //     autoware::universe_utils::createMarkerScale(0.01, 0.0, 0.0),
  //     autoware::universe_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));
  //
  //   for (const auto & detection_polygon : debug_data_ptr_->detection_polygons) {
  //     for (size_t dp_idx = 0; dp_idx < detection_polygon.outer().size(); ++dp_idx) {
  //       const auto & current_point = detection_polygon.outer().at(dp_idx);
  //       const auto & next_point =
  //         detection_polygon.outer().at((dp_idx + 1) % detection_polygon.outer().size());
  //
  //       marker.points.push_back(
  //         autoware::universe_utils::createPoint(current_point.x(), current_point.y(), 0.0));
  //       marker.points.push_back(
  //         autoware::universe_utils::createPoint(next_point.x(), next_point.y(), 0.0));
  //     }
  //   }
  //   debug_marker.markers.push_back(marker);
  // }

  // 3. print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_calculation_time_info_, "  %s := %f [ms]",
    __func__, calculation_time);
}

void ObstacleCruisePlannerNode::publishCalculationTime(const double calculation_time) const
{
  Float64Stamped calculation_time_msg;
  calculation_time_msg.stamp = now();
  calculation_time_msg.data = calculation_time;
  debug_calculation_time_pub_->publish(calculation_time_msg);
}
}  // namespace autoware::motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion_planning::ObstacleCruisePlannerNode)
