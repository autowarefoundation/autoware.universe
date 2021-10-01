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

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "pcl/filters/voxel_grid.h"

#include "diagnostic_msgs/msg/key_value.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"

#include "obstacle_stop_planner/node.hpp"

namespace motion_planning
{

using autoware_utils::calcAzimuthAngle;
using autoware_utils::calcDistance2d;
using autoware_utils::calcSignedArcLength;
using autoware_utils::createPoint;
using autoware_utils::findNearestIndex;
using autoware_utils::getRPY;

namespace
{
TrajectoryPoint getBackwardPointFromBasePoint(
  const TrajectoryPoint & p_from, const TrajectoryPoint & p_to,
  const TrajectoryPoint & p_base, const double backward_length)
{
  TrajectoryPoint output;
  const double dx = p_to.pose.position.x - p_from.pose.position.x;
  const double dy = p_to.pose.position.y - p_from.pose.position.y;
  const double norm = std::hypot(dx, dy);

  output = p_base;
  output.pose.position.x += backward_length * dx / norm;
  output.pose.position.y += backward_length * dy / norm;

  return output;
}
boost::optional<std::pair<size_t, TrajectoryPoint>> getForwardInsertPointFromBasePoint(
  const size_t base_idx, const Trajectory & trajectory, const double margin)
{
  if (base_idx + 1 > trajectory.points.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, trajectory.points.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; i < trajectory.points.size() - 1; ++i) {
    const auto & p_front = trajectory.points.at(i);
    const auto & p_back = trajectory.points.at(i + 1);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert =
        getBackwardPointFromBasePoint(p_back, p_front, p_back, length_residual);

      // p_front(trajectory.points.at(i)) is insert base point
      return std::make_pair(i, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(trajectory.points.size() - 1, trajectory.points.back());
  }

  return {};
}
boost::optional<std::pair<size_t, TrajectoryPoint>> getBackwardInsertPointFromBasePoint(
  const size_t base_idx, const Trajectory & trajectory, const double margin)
{
  if (base_idx + 1 > trajectory.points.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, trajectory.points.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; 0 < i; --i) {
    const auto & p_front = trajectory.points.at(i - 1);
    const auto & p_back = trajectory.points.at(i);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert =
        getBackwardPointFromBasePoint(p_front, p_back, p_front, length_residual);

      // p_front(trajectory.points.at(i-1)) is insert base point
      return std::make_pair(i - 1, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(size_t(0), trajectory.points.front());
  }

  return {};
}
boost::optional<std::pair<size_t, double>> findNearestFrontIndex(
  const Trajectory & trajectory, const geometry_msgs::msg::Point & point)
{
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & p_traj = trajectory.points.at(i).pose;
    const auto yaw = getRPY(p_traj).z;
    const Point2d p_traj_direction(std::cos(yaw), std::sin(yaw));
    const Point2d p_traj_to_target(point.x - p_traj.position.x, point.y - p_traj.position.y);

    const auto is_in_front_of_target_point = p_traj_direction.dot(p_traj_to_target) < 0.0;
    const auto is_trajectory_end = i + 1 == trajectory.points.size();

    if (is_in_front_of_target_point || is_trajectory_end) {
      const auto dist_p_traj_to_target = p_traj_direction.normalized().dot(p_traj_to_target);
      return std::make_pair(i, dist_p_traj_to_target);
    }
  }

  return {};
}
bool isInFrontOfTargetPoint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & point)
{
  const auto yaw = getRPY(pose).z;
  const Point2d pose_direction(std::cos(yaw), std::sin(yaw));
  const Point2d to_target(point.x - pose.position.x, point.y - pose.position.y);

  return pose_direction.dot(to_target) < 0.0;
}
std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
      R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
    pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
    pose.orientation.y % pose.orientation.z)
    .str();
  return json_dumps_pose;
}
diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
  const std::string stop_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;
  diagnostic_msgs::msg::KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}
}  // namespace

ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_stop_planner", node_options)
{
  // Vehicle Parameters
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  const auto & i = vehicle_info_;

  // Parameters
  {
    auto & p = node_param_;
    p.enable_slow_down = declare_parameter("enable_slow_down", false);
  }

  {
    auto & p = stop_param_;
    const std::string ns = "stop_planner.";
    p.stop_margin = declare_parameter(ns + "stop_margin", 5.0);
    p.min_behavior_stop_margin = declare_parameter(ns + "min_behavior_stop_margin", 2.0);
    p.expand_stop_range = declare_parameter(ns + "expand_stop_range", 0.0);
    p.extend_distance = declare_parameter(ns + "extend_distance", 0.0);
    p.step_length = declare_parameter(ns + "step_length", 1.0);
    p.stop_margin += i.max_longitudinal_offset_m;
    p.min_behavior_stop_margin += i.max_longitudinal_offset_m;
    p.stop_search_radius =
      p.step_length + std::hypot(
      i.vehicle_width_m / 2.0 + p.expand_stop_range, i.vehicle_length_m / 2.0);
  }

  {
    auto & p = slow_down_param_;
    const std::string ns = "slow_down_planner.";
    p.slow_down_forward_margin = declare_parameter(ns + "slow_down_forward_margin", 5.0);
    p.slow_down_backward_margin = declare_parameter(ns + "slow_down_backward_margin", 5.0);
    p.expand_slow_down_range = declare_parameter(ns + "expand_slow_down_range", 1.0);
    p.max_slow_down_vel = declare_parameter(ns + "max_slow_down_vel", 4.0);
    p.min_slow_down_vel = declare_parameter(ns + "min_slow_down_vel", 2.0);
    p.slow_down_forward_margin += i.max_longitudinal_offset_m;
    p.slow_down_backward_margin += i.rear_overhang_m;
    p.slow_down_search_radius = stop_param_.step_length +
      std::hypot(i.vehicle_width_m / 2.0 + p.expand_slow_down_range, i.vehicle_length_m / 2.0);
  }

  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(
    this, i.max_longitudinal_offset_m);

  // Initializer
  acc_controller_ = std::make_unique<motion_planning::AdaptiveCruiseController>(
    this, i.vehicle_width_m, i.vehicle_length_m, i.max_longitudinal_offset_m);

  // Publishers
  path_pub_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/stop_reason", 1);

  // Subscribers
  obstacle_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleStopPlannerNode::obstaclePointcloudCallback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::pathCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/twist", 1,
    std::bind(&ObstacleStopPlannerNode::currentVelocityCallback, this, std::placeholders::_1));
  dynamic_object_sub_ =
    this->create_subscription<DynamicObjectArray>(
    "~/input/objects", 1,
    std::bind(&ObstacleStopPlannerNode::dynamicObjectCallback, this, std::placeholders::_1));
  expand_stop_range_sub_ = this->create_subscription<ExpandStopRange>(
    "~/input/expand_stop_range", 1,
    std::bind(
      &ObstacleStopPlannerNode::externalExpandStopRangeCallback, this, std::placeholders::_1));
}

void ObstacleStopPlannerNode::obstaclePointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  obstacle_ros_pointcloud_ptr_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_height_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_height_filtered_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);

  for (const auto & point : pointcloud_ptr->points) {
    no_height_pointcloud_ptr->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
  }
  filter.setInputCloud(no_height_pointcloud_ptr);
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);
  filter.filter(*no_height_filtered_pointcloud_ptr);
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
}

void ObstacleStopPlannerNode::pathCallback(const Trajectory::ConstSharedPtr input_msg)
{
  if (!obstacle_ros_pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_ptr_ && node_param_.enable_slow_down) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  // extend trajectory to consider obstacles after the goal
  Trajectory extended_trajectory{};
  extendTrajectory(*input_msg, stop_param_.extend_distance, extended_trajectory);
  const auto base_trajectory{extended_trajectory};

  if (base_trajectory.points.empty()) {
    return;
  }

  Trajectory output_trajectory{base_trajectory};
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag{};

  // trim trajectory from self pose
  geometry_msgs::msg::Pose self_pose{};
  getSelfPose(input_msg->header, tf_buffer_, self_pose);
  Trajectory trim_trajectory{};
  size_t trajectory_trim_index{};
  trimTrajectoryWithIndexFromSelfPose(
    base_trajectory, self_pose, trim_trajectory, trajectory_trim_index);

  // decimate trajectory for calculation cost
  Trajectory decimate_trajectory;
  std::map<size_t /* decimate */, size_t /* origin */> decimate_trajectory_index_map;
  decimateTrajectory(
    trim_trajectory, stop_param_.step_length, decimate_trajectory, decimate_trajectory_index_map);

  // search candidate obstacle pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  if (!searchPointcloudNearTrajectory(
      decimate_trajectory, obstacle_ros_pointcloud_ptr_, obstacle_candidate_pointcloud_ptr))
  {
    return;
  }

  // for collision
  bool found_collision_points = false;
  bool stop_require = false;
  size_t decimate_trajectory_collision_index = 0;
  pcl::PointXYZ nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  // for slow down
  bool found_slow_down_points = false;
  bool slow_down_require = false;
  pcl::PointXYZ nearest_slow_down_point;
  pcl::PointXYZ lateral_nearest_slow_down_point;
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double lateral_deviation = 0.0;

  for (int i = 0; i < static_cast<int>(decimate_trajectory.points.size()) - 1; ++i) {
    // create one step circle center for vehicle
    const auto & p_front = decimate_trajectory.points.at(i).pose;
    const auto & p_back = decimate_trajectory.points.at(i + 1).pose;
    const auto prev_center_pose = getVehicleCenterFromBase(p_front);
    const Point2d prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = getVehicleCenterFromBase(p_back);
    const Point2d next_center_point(next_center_pose.position.x, next_center_pose.position.y);

    if (node_param_.enable_slow_down) {
      std::vector<cv::Point2d> one_step_move_slow_down_range_polygon;
      // create one step polygon for slow_down range
      createOneStepPolygon(
        p_front, p_back, one_step_move_slow_down_range_polygon,
        slow_down_param_.expand_slow_down_range);
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, p_front.position.z, PolygonType::SlowDownRange);

      found_slow_down_points = withinPolygon(
        one_step_move_slow_down_range_polygon, slow_down_param_.slow_down_search_radius,
        prev_center_point, next_center_point,
        obstacle_candidate_pointcloud_ptr, slow_down_pointcloud_ptr);

      const auto found_first_slow_down_points = found_slow_down_points && !slow_down_require;

      if (found_first_slow_down_points) {
        // found nearest slow down obstacle
        slow_down_require = true;
        getNearestPoint(
          *slow_down_pointcloud_ptr, p_front, &nearest_slow_down_point,
          &nearest_collision_point_time);
        getLateralNearestPoint(
          *slow_down_pointcloud_ptr, p_front, &lateral_nearest_slow_down_point, &lateral_deviation);

        debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
        debug_ptr_->pushPolygon(
          one_step_move_slow_down_range_polygon, p_front.position.z, PolygonType::SlowDown);
      }

    } else {
      slow_down_pointcloud_ptr = obstacle_candidate_pointcloud_ptr;
    }

    {
      std::vector<cv::Point2d> one_step_move_vehicle_polygon;
      // create one step polygon for vehicle
      createOneStepPolygon(
        p_front, p_back, one_step_move_vehicle_polygon, stop_param_.expand_stop_range);
      debug_ptr_->pushPolygon(
        one_step_move_vehicle_polygon, decimate_trajectory.points.at(i).pose.position.z,
        PolygonType::Vehicle);

      pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
      collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

      found_collision_points = withinPolygon(
        one_step_move_vehicle_polygon, stop_param_.stop_search_radius,
        prev_center_point, next_center_point,
        slow_down_pointcloud_ptr, collision_pointcloud_ptr);

      if (found_collision_points) {
        decimate_trajectory_collision_index = i;
        getNearestPoint(
          *collision_pointcloud_ptr, p_front,
          &nearest_collision_point, &nearest_collision_point_time);

        debug_ptr_->pushObstaclePoint(nearest_collision_point, PointType::Stop);
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon, p_front.position.z, PolygonType::Collision);

        stop_require = found_collision_points;
        acc_controller_->insertAdaptiveCruiseVelocity(
          decimate_trajectory, decimate_trajectory_collision_index, self_pose,
          nearest_collision_point, nearest_collision_point_time, object_ptr_, current_velocity_ptr_,
          &stop_require, &output_trajectory);

        break;
      }
    }
  }

  if (stop_require) {
    // insert stop point
    const auto index_with_dist_remain = findNearestFrontIndex(
      base_trajectory, createPoint(nearest_collision_point.x, nearest_collision_point.y, 0));

    if (index_with_dist_remain) {
      const auto stop_point = searchInsertPoint(
        index_with_dist_remain.get().first, base_trajectory, index_with_dist_remain.get().second);
      insertStopPoint(stop_point, output_trajectory, stop_reason_diag);
    }
  }

  if (slow_down_require) {
    // insert slow down point
    const auto index_with_dist_remain = findNearestFrontIndex(
      base_trajectory, createPoint(nearest_slow_down_point.x, nearest_slow_down_point.y, 0));

    if (index_with_dist_remain) {
      const auto slow_down_section = createSlowDownSection(
        index_with_dist_remain.get().first, base_trajectory,
        lateral_deviation, index_with_dist_remain.get().second);

      const auto dist_baselink_to_obstacle =
        calcSignedArcLength(
        base_trajectory.points, trajectory_trim_index,
        index_with_dist_remain.get().first) -
        index_with_dist_remain.get().second;

      if (!latest_slow_down_section_ &&
        dist_baselink_to_obstacle < vehicle_info_.max_longitudinal_offset_m)
      {
        latest_slow_down_section_ = slow_down_section;
      }

      insertSlowDownSection(slow_down_section, output_trajectory);
    }
  }

  if (node_param_.enable_slow_down && latest_slow_down_section_) {
    // check whether ego is in slow down section or not
    const auto & p_start = latest_slow_down_section_.get().start_point.pose.position;
    const auto & p_end = latest_slow_down_section_.get().end_point.pose.position;
    const auto reach_slow_down_start_point = isInFrontOfTargetPoint(self_pose, p_start);
    const auto reach_slow_down_end_point = isInFrontOfTargetPoint(self_pose, p_end);
    const auto is_in_slow_down_section = reach_slow_down_start_point && !reach_slow_down_end_point;
    const auto index_with_dist_remain = findNearestFrontIndex(base_trajectory, p_end);

    if (is_in_slow_down_section && index_with_dist_remain) {
      const auto end_insert_point_with_idx = getBackwardInsertPointFromBasePoint(
        index_with_dist_remain.get().first, base_trajectory, -index_with_dist_remain.get().second);

      SlowDownSection slow_down_section{};
      slow_down_section.slow_down_start_idx = 0;
      slow_down_section.start_point = output_trajectory.points.front();
      slow_down_section.slow_down_end_idx = end_insert_point_with_idx.get().first;
      slow_down_section.end_point = end_insert_point_with_idx.get().second;
      slow_down_section.velocity = latest_slow_down_section_.get().velocity;

      insertSlowDownSection(slow_down_section, output_trajectory);
    } else {
      latest_slow_down_section_ = {};
    }
  }

  path_pub_->publish(output_trajectory);
  stop_reason_diag_pub_->publish(stop_reason_diag);
  debug_ptr_->publish();
}

bool ObstacleStopPlannerNode::withinPolygon(
  const std::vector<cv::Point2d> & cv_polygon, const double radius,
  const Point2d & prev_point, const Point2d & next_point,
  pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr within_points_ptr)
{
  Polygon2d boost_polygon;
  bool find_within_points = false;
  for (const auto & point : cv_polygon) {
    boost_polygon.outer().push_back(bg::make<Point2d>(point.x, point.y));
  }
  boost_polygon.outer().push_back(bg::make<Point2d>(cv_polygon.front().x, cv_polygon.front().y));

  for (size_t j = 0; j < candidate_points_ptr->size(); ++j) {
    Point2d point(candidate_points_ptr->at(j).x, candidate_points_ptr->at(j).y);
    if (bg::distance(prev_point, point) < radius || bg::distance(next_point, point) < radius) {
      if (bg::within(point, boost_polygon)) {
        within_points_ptr->push_back(candidate_points_ptr->at(j));
        find_within_points = true;
      }
    }
  }
  return find_within_points;
}

void ObstacleStopPlannerNode::externalExpandStopRangeCallback(
  const ExpandStopRange::ConstSharedPtr input_msg)
{
  const auto & i = vehicle_info_;
  stop_param_.expand_stop_range = input_msg->expand_stop_range;
  stop_param_.stop_search_radius =
    stop_param_.step_length + std::hypot(
    i.vehicle_width_m / 2.0 + stop_param_.expand_stop_range,
    i.vehicle_length_m / 2.0);
}

void ObstacleStopPlannerNode::insertStopPoint(
  const StopPoint & stop_point, Trajectory & output,
  diagnostic_msgs::msg::DiagnosticStatus & stop_reason_diag)
{
  const auto traj_end_idx = output.points.size() - 1;
  const auto & stop_idx = stop_point.index;

  const auto & p_base = output.points.at(stop_idx);
  const auto & p_next = output.points.at(std::min(stop_idx + 1, traj_end_idx));
  const auto & p_insert = stop_point.point;

  constexpr double min_dist = 1e-3;

  const auto is_p_base_and_p_insert_overlap =
    calcDistance2d(p_base, p_insert) < min_dist;
  const auto is_p_next_and_p_insert_overlap =
    calcDistance2d(p_next, p_insert) < min_dist;

  auto update_stop_idx = stop_idx;

  if (!is_p_base_and_p_insert_overlap &&
    !is_p_next_and_p_insert_overlap)
  {
    // insert: start_idx and end_idx are shifted by one
    output.points.insert(output.points.begin() + stop_idx, p_insert);
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  } else if (is_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  }

  for (size_t i = update_stop_idx; i < output.points.size(); ++i) {
    output.points.at(i).twist.linear.x = 0.0;
  }

  stop_reason_diag = makeStopReasonDiag("obstacle", p_insert.pose);
  debug_ptr_->pushPose(p_insert.pose, PoseType::Stop);
}

StopPoint ObstacleStopPlannerNode::searchInsertPoint(
  const int idx, const Trajectory & base_trajectory, const double dist_remain)
{
  const auto max_dist_stop_point = createTargetPoint(
    idx, stop_param_.stop_margin, base_trajectory, dist_remain);
  const auto min_dist_stop_point = createTargetPoint(
    idx, stop_param_.min_behavior_stop_margin, base_trajectory, dist_remain);

  // check if stop point is already inserted by behavior planner
  bool is_inserted_already_stop_point = false;
  for (int j = max_dist_stop_point.index - 1; j < static_cast<int>(idx); ++j) {
    if (base_trajectory.points.at(std::max(j, 0)).twist.linear.x == 0.0) {
      is_inserted_already_stop_point = true;
      break;
    }
  }
  // insert stop point
  StopPoint stop_point{};
  stop_point.index =
    !is_inserted_already_stop_point ? max_dist_stop_point.index : min_dist_stop_point.index;
  stop_point.point =
    !is_inserted_already_stop_point ? max_dist_stop_point.point : min_dist_stop_point.point;
  return stop_point;
}

StopPoint ObstacleStopPlannerNode::createTargetPoint(
  const int idx, const double margin,
  const Trajectory & base_trajectory, const double dist_remain)
{
  const auto update_margin_from_vehicle = margin - dist_remain;
  const auto insert_point_with_idx = getBackwardInsertPointFromBasePoint(
    idx, base_trajectory, update_margin_from_vehicle);

  if (!insert_point_with_idx) {
    // TODO(Satoshi Ota)
    return StopPoint{};
  }

  StopPoint stop_point{};
  stop_point.index = insert_point_with_idx.get().first;
  stop_point.point = insert_point_with_idx.get().second;

  return stop_point;
}

SlowDownSection ObstacleStopPlannerNode::createSlowDownSection(
  const int idx, const Trajectory & base_trajectory,
  const double lateral_deviation, const double dist_remain)
{
  // calc slow down start point
  const auto update_forward_margin_from_vehicle =
    slow_down_param_.slow_down_forward_margin - dist_remain;
  const auto start_insert_point_with_idx = getBackwardInsertPointFromBasePoint(
    idx, base_trajectory, update_forward_margin_from_vehicle);

  // calc slow down end point
  const auto update_backward_margin_from_vehicle =
    slow_down_param_.slow_down_backward_margin + dist_remain;
  const auto end_insert_point_with_idx = getForwardInsertPointFromBasePoint(
    idx, base_trajectory, update_backward_margin_from_vehicle);

  if (!start_insert_point_with_idx || !end_insert_point_with_idx) {
    // TODO(Satoshi Ota)
    return SlowDownSection{};
  }

  SlowDownSection slow_down_section{};
  slow_down_section.slow_down_start_idx = start_insert_point_with_idx.get().first;
  slow_down_section.start_point = start_insert_point_with_idx.get().second;
  slow_down_section.slow_down_end_idx = end_insert_point_with_idx.get().first;
  slow_down_section.end_point = end_insert_point_with_idx.get().second;

  slow_down_section.velocity =
    slow_down_param_.min_slow_down_vel +
    (slow_down_param_.max_slow_down_vel - slow_down_param_.min_slow_down_vel) *
    std::max(lateral_deviation - vehicle_info_.vehicle_width_m / 2, 0.0) /
    slow_down_param_.expand_slow_down_range;

  return slow_down_section;
}

void ObstacleStopPlannerNode::insertSlowDownSection(
  const SlowDownSection & slow_down_section, Trajectory & output)
{
  const auto traj_end_idx = output.points.size() - 1;
  const auto & start_idx = slow_down_section.slow_down_start_idx;
  const auto & end_idx = slow_down_section.slow_down_end_idx;

  const auto & p_base_start = output.points.at(start_idx);
  const auto & p_next_start = output.points.at(std::min(start_idx + 1, traj_end_idx));
  const auto & p_insert_start = slow_down_section.start_point;

  const auto & p_base_end = output.points.at(end_idx);
  const auto & p_next_end = output.points.at(std::min(end_idx + 1, traj_end_idx));
  const auto & p_insert_end = slow_down_section.end_point;

  constexpr double min_dist = 1e-3;

  const auto is_start_p_base_and_p_insert_overlap =
    calcDistance2d(p_base_start, p_insert_start) < min_dist;
  const auto is_start_p_next_and_p_insert_overlap =
    calcDistance2d(p_next_start, p_insert_start) < min_dist;

  auto update_start_idx = start_idx;
  auto update_end_idx = end_idx;

  if (!is_start_p_base_and_p_insert_overlap &&
    !is_start_p_next_and_p_insert_overlap)
  {
    // insert: start_idx and end_idx are shifted by one
    output.points.insert(output.points.begin() + start_idx, p_insert_start);
    update_start_idx = std::min(update_start_idx + 1, traj_end_idx);
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  } else if (is_start_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_start_idx = std::min(update_start_idx + 1, traj_end_idx);
  }

  const auto is_end_p_base_and_p_insert_overlap =
    calcDistance2d(p_base_end, p_insert_end) < min_dist;
  const auto is_end_p_next_and_p_insert_overlap =
    calcDistance2d(p_next_end, p_insert_end) < min_dist;

  if (!is_end_p_base_and_p_insert_overlap &&
    !is_end_p_next_and_p_insert_overlap)
  {
    // insert: end_idx is shifted by one
    output.points.insert(output.points.begin() + end_idx, p_insert_end);
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  } else if (is_end_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  }

  for (size_t i = update_start_idx; i <= update_end_idx; ++i) {
    output.points.at(i).twist.linear.x =
      std::min(slow_down_section.velocity, output.points.at(i).twist.linear.x);
  }

  debug_ptr_->pushPose(p_base_start.pose, PoseType::SlowDownStart);
  debug_ptr_->pushPose(p_base_end.pose, PoseType::SlowDownEnd);
}

void ObstacleStopPlannerNode::dynamicObjectCallback(
  const DynamicObjectArray::ConstSharedPtr input_msg)
{
  object_ptr_ = input_msg;
}

void ObstacleStopPlannerNode::currentVelocityCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

TrajectoryPoint ObstacleStopPlannerNode::getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point)
{
  tf2::Transform map2goal;
  tf2::fromMsg(goal_point.pose, map2goal);
  tf2::Transform local_extend_point;
  local_extend_point.setOrigin(tf2::Vector3(extend_distance, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  local_extend_point.setRotation(q);
  const auto map2extend_point = map2goal * local_extend_point;
  geometry_msgs::msg::Pose extend_pose;
  tf2::toMsg(map2extend_point, extend_pose);
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.twist = goal_point.twist;
  extend_trajectory_point.accel = goal_point.accel;
  return extend_trajectory_point;
}

void ObstacleStopPlannerNode::extendTrajectory(
  const Trajectory & input, const double extend_distance, Trajectory & output)
{
  output = input;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return;
  }

  const auto goal_point = input.points.back();
  double interpolation_distance = 0.1;

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - interpolation_distance)) {
    const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_sum, goal_point);
    output.points.push_back(extend_trajectory_point);
    extend_sum += interpolation_distance;
  }
  const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_distance, goal_point);
  output.points.push_back(extend_trajectory_point);
}

bool ObstacleStopPlannerNode::decimateTrajectory(
  const Trajectory & input, const double step_length, Trajectory & output,
  std::map<size_t /* decimate */, size_t /* origin */> & index_map)
{
  output.header = input.header;
  double trajectory_length_sum = 0.0;
  double next_length = 0.0;

  for (int i = 0; i < static_cast<int>(input.points.size()) - 1; ++i) {
    const auto & p_front = input.points.at(i);
    const auto & p_back = input.points.at(i + 1);
    constexpr double epsilon = 1e-3;

    if (next_length <= trajectory_length_sum + epsilon) {
      const auto p_interpolate = getBackwardPointFromBasePoint(
        p_front, p_back, p_back, next_length - trajectory_length_sum);
      output.points.push_back(p_interpolate);

      index_map.insert(std::make_pair(output.points.size() - 1, size_t(i)));
      next_length += step_length;
      continue;
    }

    trajectory_length_sum += calcDistance2d(p_front, p_back);
  }
  if (!input.points.empty()) {
    output.points.push_back(input.points.back());
    index_map.insert(
      std::make_pair(output.points.size() - 1, input.points.size() - 1));
  }
  return true;
}

bool ObstacleStopPlannerNode::trimTrajectoryWithIndexFromSelfPose(
  const Trajectory & input, const geometry_msgs::msg::Pose & self_pose,
  Trajectory & output, size_t & index)
{
  double min_distance = 0.0;
  size_t min_distance_index = 0;
  bool is_init = false;
  for (size_t i = 0; i < input.points.size(); ++i) {
    const double x = input.points.at(i).pose.position.x - self_pose.position.x;
    const double y = input.points.at(i).pose.position.y - self_pose.position.y;
    const double squared_distance = x * x + y * y;
    if (!is_init || squared_distance < min_distance * min_distance) {
      is_init = true;
      min_distance = std::sqrt(squared_distance);
      min_distance_index = i;
    }
  }
  for (size_t i = min_distance_index; i < input.points.size(); ++i) {
    output.points.push_back(input.points.at(i));
  }
  output.header = input.header;
  index = min_distance_index;
  return true;
}

bool ObstacleStopPlannerNode::searchPointcloudNearTrajectory(
  const Trajectory & trajectory,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_points_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_points_ptr)
{
  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      trajectory.header.frame_id, input_points_ptr->header.frame_id, input_points_ptr->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Failed to look up transform from " <<
        trajectory.header.frame_id << " to " << input_points_ptr->header.frame_id);
    return false;
  }

  sensor_msgs::msg::PointCloud2 transformed_points{};
  const Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(affine_matrix, *input_points_ptr, transformed_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_points, *transformed_points_ptr);

  output_points_ptr->header = transformed_points_ptr->header;

  // search obstacle candidate pointcloud to reduce calculation cost
  const double search_radius =
    node_param_.enable_slow_down ? slow_down_param_.slow_down_search_radius :
    stop_param_.stop_search_radius;
  const double squared_radius = search_radius * search_radius;
  for (const auto & trajectory_point : trajectory.points) {
    const auto center_pose = getVehicleCenterFromBase(trajectory_point.pose);
    for (const auto & point : transformed_points_ptr->points) {
      const double x = center_pose.position.x - point.x;
      const double y = center_pose.position.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) {output_points_ptr->points.push_back(point);}
    }
  }
  return true;
}

void ObstacleStopPlannerNode::createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  std::vector<cv::Point2d> & polygon, const double expand_width)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;

  const auto & i = vehicle_info_;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0 + expand_width;
  const auto & back_m = i.rear_overhang_m;
  // start step
  {
    const auto yaw = getRPY(base_step_pose).z;
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
        base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
        base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
        base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
        base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  // next step
  {
    const auto yaw = getRPY(next_step_pose).z;
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
        next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
        next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
        next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
        next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  convexHull(one_step_move_vehicle_corner_points, polygon);
}

bool ObstacleStopPlannerNode::convexHull(
  const std::vector<cv::Point2d> & pointcloud, std::vector<cv::Point2d> & polygon_points)
{
  cv::Point2d centroid;
  centroid.x = 0;
  centroid.y = 0;
  for (const auto & point : pointcloud) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x = centroid.x / static_cast<double>(pointcloud.size());
  centroid.y = centroid.y / static_cast<double>(pointcloud.size());

  std::vector<cv::Point> normalized_pointcloud;
  std::vector<cv::Point> normalized_polygon_points;
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    normalized_pointcloud.push_back(
      cv::Point(
        (pointcloud.at(i).x - centroid.x) * 1000.0, (pointcloud.at(i).y - centroid.y) * 1000.0));
  }
  cv::convexHull(normalized_pointcloud, normalized_polygon_points);

  for (size_t i = 0; i < normalized_polygon_points.size(); ++i) {
    cv::Point2d polygon_point;
    polygon_point.x = (normalized_polygon_points.at(i).x / 1000.0 + centroid.x);
    polygon_point.y = (normalized_polygon_points.at(i).y / 1000.0 + centroid.y);
    polygon_points.push_back(polygon_point);
  }
  return true;
}

bool ObstacleStopPlannerNode::getSelfPose(
  const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::Pose & self_pose)
{
  try {
    geometry_msgs::msg::TransformStamped transform;
    transform =
      tf_buffer.lookupTransform(
      header.frame_id, "base_link", header.stamp, rclcpp::Duration::from_seconds(
        0.1));
    self_pose.position.x = transform.transform.translation.x;
    self_pose.position.y = transform.transform.translation.y;
    self_pose.position.z = transform.transform.translation.z;
    self_pose.orientation.x = transform.transform.rotation.x;
    self_pose.orientation.y = transform.transform.rotation.y;
    self_pose.orientation.z = transform.transform.rotation.z;
    self_pose.orientation.w = transform.transform.rotation.w;
    return true;
  } catch (tf2::TransformException & ex) {
    return false;
  }
}

void ObstacleStopPlannerNode::getNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
  pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time)
{
  double min_norm = 0.0;
  bool is_init = false;
  const auto yaw = getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));

  for (size_t i = 0; i < pointcloud.size(); ++i) {
    const Eigen::Vector2d pointcloud_vec(
      pointcloud.at(i).x - base_pose.position.x, pointcloud.at(i).y - base_pose.position.y);
    double norm = base_pose_vec.dot(pointcloud_vec);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = pointcloud.at(i);
      *nearest_collision_point_time = pcl_conversions::fromPCL(pointcloud.header).stamp;
      is_init = true;
    }
  }
}

void ObstacleStopPlannerNode::getLateralNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
  pcl::PointXYZ * lateral_nearest_point, double * deviation)
{
  double min_norm = std::numeric_limits<double>::max();
  const auto yaw = getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    const Eigen::Vector2d pointcloud_vec(
      pointcloud.at(i).x - base_pose.position.x, pointcloud.at(i).y - base_pose.position.y);
    double norm =
      std::abs(base_pose_vec.x() * pointcloud_vec.y() - base_pose_vec.y() * pointcloud_vec.x());
    if (norm < min_norm) {
      min_norm = norm;
      *lateral_nearest_point = pointcloud.at(i);
    }
  }
  *deviation = min_norm;
}

geometry_msgs::msg::Pose ObstacleStopPlannerNode::getVehicleCenterFromBase(
  const geometry_msgs::msg::Pose & base_pose)
{
  const auto & i = vehicle_info_;
  const auto yaw = getRPY(base_pose).z;

  geometry_msgs::msg::Pose center_pose;
  center_pose.position.x =
    base_pose.position.x + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::cos(yaw);
  center_pose.position.y =
    base_pose.position.y + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::sin(yaw);
  center_pose.position.z = base_pose.position.z;
  center_pose.orientation = base_pose.orientation;
  return center_pose;
}

}  // namespace motion_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleStopPlannerNode)
