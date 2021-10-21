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

#include "autoware_utils/geometry/geometry.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "obstacle_stop_planner/node.hpp"

namespace
{
double getYawFromGeometryMsgsQuaternion(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

  return yaw;
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

namespace motion_planning
{
ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_stop_planner", node_options)
{
  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  rear_overhang_ = vehicle_info.rear_overhang_m;
  left_overhang_ = vehicle_info.left_overhang_m;
  right_overhang_ = vehicle_info.right_overhang_m;
  vehicle_width_ = vehicle_info.vehicle_width_m;
  vehicle_length_ = vehicle_info.vehicle_length_m;
  baselink2front_ = vehicle_info.max_longitudinal_offset_m;

  // Parameters
  stop_margin_ = declare_parameter("stop_planner.stop_margin", 5.0);
  min_behavior_stop_margin_ = declare_parameter("stop_planner.min_behavior_stop_margin", 2.0);
  step_length_ = declare_parameter("stop_planner.step_length", 1.0);
  extend_distance_ = declare_parameter("stop_planner.extend_distance", 0.0);
  expand_stop_range_ = declare_parameter("stop_planner.expand_stop_range", 0.0);

  slow_down_forward_margin_ = declare_parameter("slow_down_planner.slow_down_forward_margin", 5.0);
  slow_down_backward_margin_ =
    declare_parameter("slow_down_planner.slow_down_backward_margin", 5.0);
  expand_slow_down_range_ = declare_parameter("slow_down_planner.expand_slow_down_range", 1.0);
  max_slow_down_vel_ = declare_parameter("slow_down_planner.max_slow_down_vel", 4.0);
  min_slow_down_vel_ = declare_parameter("slow_down_planner.min_slow_down_vel", 2.0);
  enable_slow_down_ = declare_parameter("enable_slow_down", false);

  stop_margin_ += wheel_base_ + front_overhang_;
  min_behavior_stop_margin_ += wheel_base_ + front_overhang_;
  slow_down_forward_margin_ += wheel_base_ + front_overhang_;
  slow_down_backward_margin_ += rear_overhang_;
  stop_search_radius_ =
    step_length_ + std::hypot(vehicle_width_ / 2.0 + expand_stop_range_, vehicle_length_ / 2.0);
  slow_down_search_radius_ =
    step_length_ +
    std::hypot(vehicle_width_ / 2.0 + expand_slow_down_range_, vehicle_length_ / 2.0);
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(this, baselink2front_);

  // Initializer
  acc_controller_ = std::make_unique<motion_planning::AdaptiveCruiseController>(
    this, vehicle_width_, vehicle_length_, baselink2front_);

  // Publishers
  path_pub_ =
    this->create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/stop_reason", 1);

  // Subscribers
  obstacle_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleStopPlannerNode::obstaclePointcloudCallback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::pathCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/twist", 1,
    std::bind(&ObstacleStopPlannerNode::currentVelocityCallback, this, std::placeholders::_1));
  dynamic_object_sub_ =
    this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "~/input/objects", 1,
    std::bind(&ObstacleStopPlannerNode::dynamicObjectCallback, this, std::placeholders::_1));
  expand_stop_range_sub_ = this->create_subscription<autoware_planning_msgs::msg::ExpandStopRange>(
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

void ObstacleStopPlannerNode::pathCallback(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg)
{
  if (!obstacle_ros_pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_ptr_ && enable_slow_down_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  // extend trajectory to consider obstacles after the goal
  autoware_planning_msgs::msg::Trajectory extended_trajectory{};
  extendTrajectory(*input_msg, extend_distance_, extended_trajectory);
  const auto base_trajectory{extended_trajectory};

  if (base_trajectory.points.empty()) {
    return;
  }

  autoware_planning_msgs::msg::Trajectory output_msg{base_trajectory};
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag{};

  // trim trajectory from self pose
  geometry_msgs::msg::Pose self_pose{};
  getSelfPose(input_msg->header, tf_buffer_, self_pose);
  autoware_planning_msgs::msg::Trajectory trim_trajectory{};
  size_t trajectory_trim_index{};
  trimTrajectoryWithIndexFromSelfPose(
    base_trajectory, self_pose, trim_trajectory, trajectory_trim_index);

  // decimate trajectory for calculation cost
  autoware_planning_msgs::msg::Trajectory decimate_trajectory;
  std::map<size_t /* decimate */, size_t /* origin */> decimate_trajectory_index_map;
  decimateTrajectory(
    trim_trajectory, step_length_, decimate_trajectory, decimate_trajectory_index_map);

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
  size_t decimate_trajectory_collision_index;
  pcl::PointXYZ nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  // for slow down
  bool found_slow_down_points = false;
  bool slow_down_require = false;
  size_t decimate_trajectory_slow_down_index;
  pcl::PointXYZ nearest_slow_down_point;
  pcl::PointXYZ lateral_nearest_slow_down_point;
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double lateral_deviation = 0.0;

  for (int i = 0; i < static_cast<int>(decimate_trajectory.points.size()) - 1; ++i) {
    // create one step circle center for vehicle
    const auto prev_center_pose = getVehicleCenterFromBase(decimate_trajectory.points.at(i).pose);
    Point prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose =
      getVehicleCenterFromBase(decimate_trajectory.points.at(i + 1).pose);
    Point next_center_point(next_center_pose.position.x, next_center_pose.position.y);

    if (enable_slow_down_) {
      std::vector<cv::Point2d> one_step_move_slow_down_range_polygon;
      // create one step polygon for slow_down range
      createOneStepPolygon(
        decimate_trajectory.points.at(i).pose, decimate_trajectory.points.at(i + 1).pose,
        one_step_move_slow_down_range_polygon, expand_slow_down_range_);
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, decimate_trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDownRange);

      found_slow_down_points = withinPolygon(
        one_step_move_slow_down_range_polygon, prev_center_point, next_center_point,
        obstacle_candidate_pointcloud_ptr, slow_down_pointcloud_ptr);
      if (!slow_down_require && found_slow_down_points) {
        // found nearest slow down obstacle
        slow_down_require = true;
        decimate_trajectory_slow_down_index = i;
        debug_ptr_->pushPolygon(
          one_step_move_slow_down_range_polygon, decimate_trajectory.points.at(i).pose.position.z,
          PolygonType::SlowDown);
        getNearestPoint(
          *slow_down_pointcloud_ptr, decimate_trajectory.points.at(i).pose,
          &nearest_slow_down_point, &nearest_collision_point_time);
        getLateralNearestPoint(
          *slow_down_pointcloud_ptr, decimate_trajectory.points.at(i).pose,
          &lateral_nearest_slow_down_point, &lateral_deviation);
        debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
      }

    } else {
      slow_down_pointcloud_ptr = obstacle_candidate_pointcloud_ptr;
    }

    {
      std::vector<cv::Point2d> one_step_move_vehicle_polygon;
      // create one step polygon for vehicle
      createOneStepPolygon(
        decimate_trajectory.points.at(i).pose, decimate_trajectory.points.at(i + 1).pose,
        one_step_move_vehicle_polygon, expand_stop_range_);
      debug_ptr_->pushPolygon(
        one_step_move_vehicle_polygon, decimate_trajectory.points.at(i).pose.position.z,
        PolygonType::Vehicle);

      pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
      collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

      found_collision_points = withinPolygon(
        one_step_move_vehicle_polygon, prev_center_point, next_center_point,
        slow_down_pointcloud_ptr, collision_pointcloud_ptr);
      if (found_collision_points) {
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon, decimate_trajectory.points.at(i).pose.position.z,
          PolygonType::Collision);
        getNearestPoint(
          *collision_pointcloud_ptr, decimate_trajectory.points.at(i).pose,
          &nearest_collision_point, &nearest_collision_point_time);
        debug_ptr_->pushObstaclePoint(nearest_collision_point, PointType::Stop);
        decimate_trajectory_collision_index = i;

        stop_require = found_collision_points;
        acc_controller_->insertAdaptiveCruiseVelocity(
          decimate_trajectory, decimate_trajectory_collision_index, self_pose,
          nearest_collision_point, nearest_collision_point_time, object_ptr_, current_velocity_ptr_,
          &stop_require, &output_msg);

        break;
      }
    }
  }

  if (stop_require) {
    // insert stop point
    for (size_t i = decimate_trajectory_index_map.at(decimate_trajectory_collision_index) +
      trajectory_trim_index;
      i < base_trajectory.points.size(); ++i)
    {
      const double yaw =
        getYawFromGeometryMsgsQuaternion(base_trajectory.points.at(i).pose.orientation);
      const Eigen::Vector2d trajectory_vec(std::cos(yaw), std::sin(yaw));
      const Eigen::Vector2d collision_point_vec(
        nearest_collision_point.x - base_trajectory.points.at(i).pose.position.x,
        nearest_collision_point.y - base_trajectory.points.at(i).pose.position.y);

      if (
        trajectory_vec.dot(collision_point_vec) < 0.0 ||
        (i + 1 == base_trajectory.points.size() && 0.0 < trajectory_vec.dot(collision_point_vec)))
      {
        const auto stop_point =
          searchInsertPoint(i, base_trajectory, trajectory_vec, collision_point_vec);
        if (stop_point.index <= output_msg.points.size()) {
          insertStopPoint(stop_point, base_trajectory, output_msg, stop_reason_diag);
        }
        break;
      }
    }
  }

  if (slow_down_require) {
    // insert slow down point
    for (size_t i = decimate_trajectory_index_map.at(decimate_trajectory_slow_down_index);
      i < base_trajectory.points.size(); ++i)
    {
      const double yaw =
        getYawFromGeometryMsgsQuaternion(base_trajectory.points.at(i).pose.orientation);
      const Eigen::Vector2d trajectory_vec(std::cos(yaw), std::sin(yaw));
      const Eigen::Vector2d slow_down_point_vec(
        nearest_slow_down_point.x - base_trajectory.points.at(i).pose.position.x,
        nearest_slow_down_point.y - base_trajectory.points.at(i).pose.position.y);

      if (
        trajectory_vec.dot(slow_down_point_vec) < 0.0 ||
        (i + 1 == base_trajectory.points.size() && 0.0 < trajectory_vec.dot(slow_down_point_vec)))
      {
        const auto slow_down_section = createSlowDownSection(
          i, lateral_deviation, trajectory_vec, slow_down_point_vec, base_trajectory);

        if (slow_down_section.slow_down_start_idx <= output_msg.points.size()) {
          insertSlowDownSection(slow_down_section, base_trajectory, output_msg);
        }
        break;
      }
    }
  }
  path_pub_->publish(output_msg);
  stop_reason_diag_pub_->publish(stop_reason_diag);
  debug_ptr_->publish();
}

bool ObstacleStopPlannerNode::withinPolygon(
  const std::vector<cv::Point2d> cv_polygon, const Point & prev_point, const Point & next_point,
  pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr within_points_ptr)
{
  Polygon boost_polygon;
  for (const auto & point : cv_polygon) {
    boost_polygon.outer().push_back(bg::make<Point>(point.x, point.y));
  }
  boost_polygon.outer().push_back(bg::make<Point>(cv_polygon.front().x, cv_polygon.front().y));

  for (size_t j = 0; j < candidate_points_ptr->size(); ++j) {
    Point point(candidate_points_ptr->at(j).x, candidate_points_ptr->at(j).y);
    if (
      bg::distance(prev_point, point) < slow_down_search_radius_ ||
      bg::distance(next_point, point) < slow_down_search_radius_)
    {
      if (bg::within(point, boost_polygon)) {
        within_points_ptr->push_back(candidate_points_ptr->at(j));
        return true;
      }
    }
  }
  return false;
}

void ObstacleStopPlannerNode::externalExpandStopRangeCallback(
  const autoware_planning_msgs::msg::ExpandStopRange::ConstSharedPtr input_msg)
{
  expand_stop_range_ = input_msg->expand_stop_range;
  stop_search_radius_ =
    step_length_ + std::hypot(vehicle_width_ / 2.0 + expand_stop_range_, vehicle_length_ / 2.0);
}

void ObstacleStopPlannerNode::insertStopPoint(
  const StopPoint & stop_point, const autoware_planning_msgs::msg::Trajectory & base_trajectory,
  autoware_planning_msgs::msg::Trajectory & output_path,
  diagnostic_msgs::msg::DiagnosticStatus & stop_reason_diag)
{
  autoware_planning_msgs::msg::TrajectoryPoint stop_trajectory_point =
    base_trajectory.points.at(std::max(static_cast<int>(stop_point.index) - 1, 0));
  stop_trajectory_point.pose.position.x = stop_point.point.x();
  stop_trajectory_point.pose.position.y = stop_point.point.y();
  stop_trajectory_point.twist.linear.x = 0.0;
  output_path.points.insert(output_path.points.begin() + stop_point.index, stop_trajectory_point);
  for (size_t j = stop_point.index; j < output_path.points.size(); ++j) {
    output_path.points.at(j).twist.linear.x = 0.0;
  }
  stop_reason_diag = makeStopReasonDiag("obstacle", stop_trajectory_point.pose);
  debug_ptr_->pushPose(stop_trajectory_point.pose, PoseType::Stop);
}

StopPoint ObstacleStopPlannerNode::searchInsertPoint(
  const int idx, const autoware_planning_msgs::msg::Trajectory & base_trajectory,
  const Eigen::Vector2d & trajectory_vec, const Eigen::Vector2d & collision_point_vec)
{
  const auto max_dist_stop_point =
    createTargetPoint(idx, stop_margin_, trajectory_vec, collision_point_vec, base_trajectory);
  const auto min_dist_stop_point = createTargetPoint(
    idx, min_behavior_stop_margin_, trajectory_vec, collision_point_vec, base_trajectory);

  // check if stop point is already inserted by behavior planner
  bool is_inserted_already_stop_point = false;
  for (int j = max_dist_stop_point.index - 1; j < static_cast<int>(idx); ++j) {
    if (base_trajectory.points.at(std::max(j, 0)).twist.linear.x == 0.0) {
      is_inserted_already_stop_point = true;
      break;
    }
  }
  // insert stop point
  StopPoint stop_point;
  stop_point.index =
    !is_inserted_already_stop_point ? max_dist_stop_point.index : min_dist_stop_point.index;
  stop_point.point =
    !is_inserted_already_stop_point ? max_dist_stop_point.point : min_dist_stop_point.point;
  return stop_point;
}

StopPoint ObstacleStopPlannerNode::createTargetPoint(
  const int idx, const double margin, const Eigen::Vector2d & trajectory_vec,
  const Eigen::Vector2d & collision_point_vec,
  const autoware_planning_msgs::msg::Trajectory & base_trajectory)
{
  double length_sum = 0.0;
  length_sum += trajectory_vec.normalized().dot(collision_point_vec);
  Eigen::Vector2d line_start_point, line_end_point;

  StopPoint stop_point{0, Eigen::Vector2d()};
  for (size_t j = idx; 0 < j; --j) {
    line_start_point << base_trajectory.points.at(j - 1).pose.position.x,
      base_trajectory.points.at(j - 1).pose.position.y;
    line_end_point << base_trajectory.points.at(j).pose.position.x,
      base_trajectory.points.at(j).pose.position.y;
    if (margin < length_sum) {
      stop_point.index = j;
      break;
    }
    length_sum += (line_end_point - line_start_point).norm();
  }
  getBackwardPointFromBasePoint(
    line_start_point, line_end_point, line_start_point, length_sum - margin, stop_point.point);

  return stop_point;
}

SlowDownSection ObstacleStopPlannerNode::createSlowDownSection(
  const int idx, const double lateral_deviation, const Eigen::Vector2d & trajectory_vec,
  const Eigen::Vector2d & slow_down_point_vec,
  const autoware_planning_msgs::msg::Trajectory & base_trajectory)
{
  SlowDownSection slow_down_section;
  double length_sum_forward = 0.0;
  double length_sum_backward = 0.0;
  double delta_length_forward = 0.0;
  double delta_length_backward = 0.0;
  length_sum_forward += trajectory_vec.normalized().dot(slow_down_point_vec);

  for (size_t i = idx; 0 < i; --i) {
    const Eigen::Vector2d p1(
      base_trajectory.points.at(i).pose.position.x, base_trajectory.points.at(i).pose.position.y);
    const Eigen::Vector2d p2(
      base_trajectory.points.at(i - 1).pose.position.x,
      base_trajectory.points.at(i - 1).pose.position.y);

    if (slow_down_forward_margin_ < length_sum_forward) {
      slow_down_section.slow_down_start_idx = i;
      getBackwardPointFromBasePoint(p1, p2, p1, delta_length_forward, slow_down_section.point);
      break;
    }
    length_sum_forward += (p1 - p2).norm();
    delta_length_forward = length_sum_forward - slow_down_forward_margin_;
  }

  if (delta_length_forward < std::numeric_limits<double>::epsilon()) {
    slow_down_section.slow_down_start_idx = 0;
    slow_down_section.point.x() = base_trajectory.points.at(0).pose.position.x;
    slow_down_section.point.y() = base_trajectory.points.at(0).pose.position.y;
  }

  for (size_t i = idx; i < base_trajectory.points.size() - 1; ++i) {
    const Eigen::Vector2d p1(
      base_trajectory.points.at(i).pose.position.x, base_trajectory.points.at(i).pose.position.y);
    const Eigen::Vector2d p2(
      base_trajectory.points.at(i + 1).pose.position.x,
      base_trajectory.points.at(i + 1).pose.position.y);

    if (slow_down_backward_margin_ < length_sum_backward) {
      slow_down_section.slow_down_end_idx = i;
      break;
    }
    length_sum_backward += (p1 - p2).norm();
    delta_length_backward = length_sum_backward - slow_down_backward_margin_;
  }

  if (delta_length_backward < std::numeric_limits<double>::epsilon()) {
    slow_down_section.slow_down_end_idx = base_trajectory.points.size() - 1;
  }

  slow_down_section.velocity =
    min_slow_down_vel_ + (max_slow_down_vel_ - min_slow_down_vel_) *
    std::max(lateral_deviation - vehicle_width_ / 2, 0.0) /
    expand_slow_down_range_;
  return slow_down_section;
}

void ObstacleStopPlannerNode::insertSlowDownSection(
  const SlowDownSection & slow_down_section,
  const autoware_planning_msgs::msg::Trajectory & base_trajectory,
  autoware_planning_msgs::msg::Trajectory & output_path)
{
  autoware_planning_msgs::msg::TrajectoryPoint slow_down_start_trajectory_point =
    base_trajectory.points.at(slow_down_section.slow_down_start_idx);
  slow_down_start_trajectory_point.pose.position.x = slow_down_section.point.x();
  slow_down_start_trajectory_point.pose.position.y = slow_down_section.point.y();
  slow_down_start_trajectory_point.twist.linear.x = slow_down_section.velocity;
  constexpr double epsilon = 0.001;
  const auto & insert_target_point = output_path.points.at(slow_down_section.slow_down_start_idx);
  if (
    autoware_utils::calcDistance2d(slow_down_start_trajectory_point, insert_target_point) >
    epsilon)
  {
    output_path.points.insert(
      output_path.points.begin() + slow_down_section.slow_down_start_idx,
      slow_down_start_trajectory_point);
  }
  debug_ptr_->pushPose(slow_down_start_trajectory_point.pose, PoseType::SlowDownStart);

  autoware_planning_msgs::msg::TrajectoryPoint slow_down_end_trajectory_point =
    output_path.points.at(slow_down_section.slow_down_end_idx);
  for (size_t i = slow_down_section.slow_down_start_idx; i <= slow_down_section.slow_down_end_idx;
    ++i)
  {
    output_path.points.at(i).twist.linear.x =
      std::min(slow_down_section.velocity, output_path.points.at(i).twist.linear.x);
  }
  debug_ptr_->pushPose(slow_down_end_trajectory_point.pose, PoseType::SlowDownEnd);
}

void ObstacleStopPlannerNode::dynamicObjectCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  object_ptr_ = input_msg;
}

void ObstacleStopPlannerNode::currentVelocityCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

autoware_planning_msgs::msg::TrajectoryPoint ObstacleStopPlannerNode::getExtendTrajectoryPoint(
  const double extend_distance, const autoware_planning_msgs::msg::TrajectoryPoint & goal_point)
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
  autoware_planning_msgs::msg::TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.twist = goal_point.twist;
  extend_trajectory_point.accel = goal_point.accel;
  return extend_trajectory_point;
}

bool ObstacleStopPlannerNode::extendTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const double extend_distance,
  autoware_planning_msgs::msg::Trajectory & output_trajectory)
{
  output_trajectory = input_trajectory;
  const auto goal_point = input_trajectory.points.back();
  double interpolation_distance = 0.1;

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - interpolation_distance)) {
    const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_sum, goal_point);
    output_trajectory.points.push_back(extend_trajectory_point);
    extend_sum += interpolation_distance;
  }
  const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_distance, goal_point);
  output_trajectory.points.push_back(extend_trajectory_point);

  return true;
}

bool ObstacleStopPlannerNode::decimateTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length,
  autoware_planning_msgs::msg::Trajectory & output_trajectory,
  std::map<size_t /* decimate */, size_t /* origin */> & index_map)
{
  output_trajectory.header = input_trajectory.header;
  double trajectory_length_sum = 0.0;
  double next_length = 0.0;
  const double epsilon = 0.001;

  for (int i = 0; i < static_cast<int>(input_trajectory.points.size()) - 1; ++i) {
    if (next_length <= trajectory_length_sum + epsilon) {
      Eigen::Vector2d interpolated_point;
      const Eigen::Vector2d line_start_point(
        input_trajectory.points.at(i).pose.position.x,
        input_trajectory.points.at(i).pose.position.y);
      const Eigen::Vector2d line_end_point(
        input_trajectory.points.at(i + 1).pose.position.x,
        input_trajectory.points.at(i + 1).pose.position.y);
      getBackwardPointFromBasePoint(
        line_start_point, line_end_point, line_end_point,
        -1.0 * (trajectory_length_sum - next_length), interpolated_point);
      autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
      trajectory_point = input_trajectory.points.at(i);
      trajectory_point.pose.position.x = interpolated_point.x();
      trajectory_point.pose.position.y = interpolated_point.y();
      output_trajectory.points.push_back(trajectory_point);
      index_map.insert(std::make_pair(output_trajectory.points.size() - 1, size_t(i)));
      next_length += step_length;
      continue;
    }
    const double x = input_trajectory.points.at(i).pose.position.x -
      input_trajectory.points.at(i + 1).pose.position.x;
    const double y = input_trajectory.points.at(i).pose.position.y -
      input_trajectory.points.at(i + 1).pose.position.y;
    const double distance = std::sqrt(x * x + y * y);

    trajectory_length_sum += distance;
  }
  if (!input_trajectory.points.empty()) {
    output_trajectory.points.push_back(input_trajectory.points.back());
    index_map.insert(
      std::make_pair(output_trajectory.points.size() - 1, input_trajectory.points.size() - 1));
  }
  return true;
}

bool ObstacleStopPlannerNode::trimTrajectoryWithIndexFromSelfPose(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const geometry_msgs::msg::Pose self_pose,
  autoware_planning_msgs::msg::Trajectory & output_trajectory, size_t & index)
{
  double min_distance = 0.0;
  size_t min_distance_index = 0;
  bool is_init = false;
  for (size_t i = 0; i < input_trajectory.points.size(); ++i) {
    const double x = input_trajectory.points.at(i).pose.position.x - self_pose.position.x;
    const double y = input_trajectory.points.at(i).pose.position.y - self_pose.position.y;
    const double squared_distance = x * x + y * y;
    if (!is_init || squared_distance < min_distance * min_distance) {
      is_init = true;
      min_distance = std::sqrt(squared_distance);
      min_distance_index = i;
    }
  }
  for (size_t i = min_distance_index; i < input_trajectory.points.size(); ++i) {
    output_trajectory.points.push_back(input_trajectory.points.at(i));
  }
  output_trajectory.header = input_trajectory.header;
  index = min_distance_index;
  return true;
}

bool ObstacleStopPlannerNode::trimTrajectoryFromSelfPose(
  [[maybe_unused]] const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const geometry_msgs::msg::Pose self_pose,
  autoware_planning_msgs::msg::Trajectory & output_trajectory)
{
  size_t index;
  return trimTrajectoryWithIndexFromSelfPose(
    output_trajectory, self_pose, output_trajectory, index);
}

bool ObstacleStopPlannerNode::searchPointcloudNearTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
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
  const double search_radius = enable_slow_down_ ? slow_down_search_radius_ : stop_search_radius_;
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
  const geometry_msgs::msg::Pose base_step_pose, const geometry_msgs::msg::Pose next_step_pose,
  std::vector<cv::Point2d> & polygon, const double expand_width)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;
  // start step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(base_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * baselink2front_ -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
        base_step_pose.position.y + std::sin(yaw) * baselink2front_ +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * baselink2front_ -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
        base_step_pose.position.y + std::sin(yaw) * baselink2front_ +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
        base_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
        base_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
  }
  // next step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(next_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * baselink2front_ -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
        next_step_pose.position.y + std::sin(yaw) * baselink2front_ +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * baselink2front_ -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
        next_step_pose.position.y + std::sin(yaw) * baselink2front_ +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
        next_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
        next_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
  }
  convexHull(one_step_move_vehicle_corner_points, polygon);
}

bool ObstacleStopPlannerNode::convexHull(
  const std::vector<cv::Point2d> pointcloud, std::vector<cv::Point2d> & polygon_points)
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
bool ObstacleStopPlannerNode::getBackwardPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

void ObstacleStopPlannerNode::getNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
  pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time)
{
  double min_norm = 0.0;
  bool is_init = false;
  const double yaw = getYawFromGeometryMsgsQuaternion(base_pose.orientation);
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
  const double yaw = getYawFromGeometryMsgsQuaternion(base_pose.orientation);
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
  geometry_msgs::msg::Pose center_pose;
  const double yaw = getYawFromGeometryMsgsQuaternion(base_pose.orientation);
  center_pose.position.x =
    base_pose.position.x + (vehicle_length_ / 2.0 - rear_overhang_) * std::cos(yaw);
  center_pose.position.y =
    base_pose.position.y + (vehicle_length_ / 2.0 - rear_overhang_) * std::sin(yaw);
  center_pose.position.z = base_pose.position.z;
  center_pose.orientation = base_pose.orientation;
  return center_pose;
}

}  // namespace motion_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleStopPlannerNode)
