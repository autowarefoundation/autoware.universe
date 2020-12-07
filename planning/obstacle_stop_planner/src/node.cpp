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
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pcl/filters/voxel_grid.h"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "obstacle_stop_planner/node.hpp"
#include <vector>
#define EIGEN_MPL2_ONLY
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
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
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

ObstacleStopPlannerNode::ObstacleStopPlannerNode() 
: Node("obstacle_stop_planner"),  tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Vehicle Parameters
  wheel_base_ = declare_parameter("/vehicle_info/wheel_base", 1.0);
  front_overhang_ = declare_parameter("/vehicle_info/front_overhang", 0.5);
  rear_overhang_ = declare_parameter("/vehicle_info/rear_overhang", 0.5);
  left_overhang_ = declare_parameter("/vehicle_info/left_overhang", 0.5);
  right_overhang_ = declare_parameter("/vehicle_info/right_overhang", 0.5);
  vehicle_width_ = declare_parameter("/vehicle_info/vehicle_width", 1.5);
  vehicle_length_ = declare_parameter("/vehicle_info/vehicle_length", 2.0);

  // Parameters
  stop_margin_ = declare_parameter("stop_margin", 5.0);
  slow_down_margin_ = declare_parameter("slow_down_margin", 5.0);
  min_behavior_stop_margin_ = declare_parameter("min_behavior_stop_margin", 2.0);
  expand_slow_down_range_ = declare_parameter("expand_slow_down_range", 1.0);
  max_slow_down_vel_ = declare_parameter("max_slow_down_vel", 4.0);
  min_slow_down_vel_ = declare_parameter("min_slow_down_vel", 2.0);
  max_deceleration_ = declare_parameter("max_deceleration", 2.0);
  enable_slow_down_ = declare_parameter("enable_slow_down", false);
  step_length_ = declare_parameter("step_length", 1.0);
  stop_margin_ += wheel_base_ + front_overhang_;
  min_behavior_stop_margin_ += wheel_base_ + front_overhang_;
  slow_down_margin_ += wheel_base_ + front_overhang_;
  stop_search_radius_ = step_length_ + std::hypot(vehicle_width_ / 2.0, vehicle_length_ / 2.0);
  slow_down_search_radius_ =
    step_length_ +
    std::hypot(vehicle_width_ / 2.0 + expand_slow_down_range_, vehicle_length_ / 2.0);
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(wheel_base_ + front_overhang_);

  // Initializer
  acc_controller_ = std::make_unique<motion_planning::AdaptiveCruiseController>(
    vehicle_width_, vehicle_length_, wheel_base_, front_overhang_);

  // Publishers
  path_pub_ = 
    this->create_publisher<autoware_planning_msgs::msg::Trajectory>("output/trajectory", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("output/stop_reason", 1);

  // Subscribers
  obstacle_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud", 1,
    std::bind(&ObstacleStopPlannerNode::obstaclePointcloudCallback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/trajectory", 1, 
    std::bind(&ObstacleStopPlannerNode::pathCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", 1,
    std::bind(&ObstacleStopPlannerNode::currentVelocityCallback, this, std::placeholders::_1)
  );
  dynamic_object_sub_ = this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "input/objects", 1,
    std::bind(&ObstacleStopPlannerNode::dynamicObjectCallback, this, std::placeholders::_1)
  );
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

  for (const auto & point : pointcloud_ptr->points)
    no_height_pointcloud_ptr->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_ptr_ && enable_slow_down_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  const autoware_planning_msgs::msg::Trajectory base_path = *input_msg;
  autoware_planning_msgs::msg::Trajectory output_msg = *input_msg;
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;
  const double epsilon = 0.00001;
  /*
   * trim trajectory from self pose
   */
  geometry_msgs::msg::Pose self_pose;
  getSelfPose(input_msg->header, tf_buffer_, self_pose);
  autoware_planning_msgs::msg::Trajectory trim_trajectory;
  size_t trajectory_trim_index;
  trimTrajectoryWithIndexFromSelfPose(
    *input_msg, self_pose, trim_trajectory, trajectory_trim_index);

  /*
   * decimate trajectory for calculation cost
   */
  autoware_planning_msgs::msg::Trajectory decimate_trajectory;
  std::map<size_t /* decimate */, size_t /* origin */> decimate_trajectory_index_map;
  decimateTrajectory(
    trim_trajectory, step_length_, decimate_trajectory, decimate_trajectory_index_map);

  autoware_planning_msgs::msg::Trajectory & trajectory = decimate_trajectory;

  /*
   * search candidate obstacle pointcloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  {
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        trajectory.header.frame_id, obstacle_ros_pointcloud_ptr_->header.frame_id,
        obstacle_ros_pointcloud_ptr_->header.stamp, rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(get_logger(),
        "[obstacle_stop_plannnr] Failed to look up transform from "
        << trajectory.header.frame_id << " to " << obstacle_ros_pointcloud_ptr_->header.frame_id);
      // do not publish path
      return;
    }

    Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pointcloud_pcl_ptr_(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*obstacle_ros_pointcloud_ptr_, *obstacle_pointcloud_pcl_ptr_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_obstacle_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*obstacle_pointcloud_pcl_ptr_, *transformed_obstacle_pointcloud_ptr, affine_matrix);

    // search obstacle candidate pointcloud to reduce calculation cost
    const double search_radius = enable_slow_down_ ? slow_down_search_radius_ : stop_search_radius_;
    searchPointcloudNearTrajectory(
      trajectory, search_radius, transformed_obstacle_pointcloud_ptr,
      obstacle_candidate_pointcloud_ptr);
    obstacle_candidate_pointcloud_ptr->header = transformed_obstacle_pointcloud_ptr->header;
  }

  /*
   * check collision, slow_down
   */
  // for collision
  bool is_collision = false;
  size_t decimate_trajectory_collision_index;
  pcl::PointXYZ nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  // for slow down
  bool canditate_slow_down = false;
  bool is_slow_down = false;
  size_t decimate_trajectory_slow_down_index;
  pcl::PointXYZ nearest_slow_down_point;
  pcl::PointXYZ lateral_nearest_slow_down_point;
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double lateral_deviation = 0.0;
  for (int i = 0; i < (int)(trajectory.points.size()) - 1; ++i) {
    /*
     * create one step circle center for vehicle
     */
    const auto prev_center_pose = getVehicleCenterFromBase(trajectory.points.at(i).pose);
    Point prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = getVehicleCenterFromBase(trajectory.points.at(i + 1).pose);
    Point next_center_point(next_center_pose.position.x, next_center_pose.position.y);
    /*
     * create one step polygon for vehicle
     */
    std::vector<cv::Point2d> one_step_move_vehicle_polygon;
    createOneStepPolygon(
      trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
      one_step_move_vehicle_polygon);
    debug_ptr_->pushPolygon(
      one_step_move_vehicle_polygon, trajectory.points.at(i).pose.position.z, PolygonType::Vehicle);
    // convert boost polygon
    Polygon boost_one_step_move_vehicle_polygon;
    for (const auto & point : one_step_move_vehicle_polygon) {
      boost_one_step_move_vehicle_polygon.outer().push_back(bg::make<Point>(point.x, point.y));
    }
    boost_one_step_move_vehicle_polygon.outer().push_back(bg::make<Point>(
      one_step_move_vehicle_polygon.front().x, one_step_move_vehicle_polygon.front().y));

    std::vector<cv::Point2d> one_step_move_slow_down_range_polygon;
    Polygon boost_one_step_move_slow_down_range_polygon;
    if (enable_slow_down_) {
      /*
      * create one step polygon for slow_down range
      */
      createOneStepPolygon(
        trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
        one_step_move_slow_down_range_polygon, expand_slow_down_range_);
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDownRange);
      // convert boost polygon
      for (const auto & point : one_step_move_slow_down_range_polygon) {
        boost_one_step_move_slow_down_range_polygon.outer().push_back(
          bg::make<Point>(point.x, point.y));
      }
      boost_one_step_move_slow_down_range_polygon.outer().push_back(bg::make<Point>(
        one_step_move_slow_down_range_polygon.front().x,
        one_step_move_slow_down_range_polygon.front().y));
    }

    // check within polygon
    pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;
    if (!is_slow_down && enable_slow_down_) {
      for (size_t j = 0; j < obstacle_candidate_pointcloud_ptr->size(); ++j) {
        Point point(
          obstacle_candidate_pointcloud_ptr->at(j).x, obstacle_candidate_pointcloud_ptr->at(j).y);
        if (
          bg::distance(prev_center_point, point) < slow_down_search_radius_ ||
          bg::distance(next_center_point, point) < slow_down_search_radius_) {
          if (bg::within(point, boost_one_step_move_slow_down_range_polygon)) {
            slow_down_pointcloud_ptr->push_back(obstacle_candidate_pointcloud_ptr->at(j));
            canditate_slow_down = true;
          }
        }
      }
    } else {
      slow_down_pointcloud_ptr = obstacle_candidate_pointcloud_ptr;
    }
    for (size_t j = 0; j < slow_down_pointcloud_ptr->size(); ++j) {
      Point point(slow_down_pointcloud_ptr->at(j).x, slow_down_pointcloud_ptr->at(j).y);
      if (
        bg::distance(prev_center_point, point) < stop_search_radius_ ||
        bg::distance(next_center_point, point) < stop_search_radius_) {
        if (bg::within(point, boost_one_step_move_vehicle_polygon)) {
          collision_pointcloud_ptr->push_back(slow_down_pointcloud_ptr->at(j));
          is_collision = true;
          debug_ptr_->pushPolygon(
            one_step_move_vehicle_polygon, trajectory.points.at(i).pose.position.z,
            PolygonType::Collision);
        }
      }
    }
    if (canditate_slow_down && !is_collision && !is_slow_down) {
      is_slow_down = true;
      decimate_trajectory_slow_down_index = i;
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDown);
      getNearestPoint(
        *slow_down_pointcloud_ptr, trajectory.points.at(i).pose, &nearest_slow_down_point,
        &nearest_collision_point_time);
      getLateralNearestPoint(
        *slow_down_pointcloud_ptr, trajectory.points.at(i).pose, &lateral_nearest_slow_down_point,
        &lateral_deviation);
      debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
    }

    /*
     * search nearest collision point by begining of path
     */
    if (is_collision) {
      getNearestPoint(
        *collision_pointcloud_ptr, trajectory.points.at(i).pose, &nearest_collision_point,
        &nearest_collision_point_time);
      debug_ptr_->pushObstaclePoint(nearest_collision_point, PointType::Stop);
      decimate_trajectory_collision_index = i;
      break;
    }
  }

  /*
   * insert max velocity and judge if there is a need to stop
   */
  bool need_to_stop = is_collision;
  if (is_collision) {
    acc_controller_->insertAdaptiveCruiseVelocity(
      decimate_trajectory, decimate_trajectory_collision_index, self_pose, nearest_collision_point,
      nearest_collision_point_time, object_ptr_, current_velocity_ptr_, &need_to_stop, &output_msg);
  }

  /*
   * insert stop point
   */
  if (need_to_stop) {
    for (int i = decimate_trajectory_index_map.at(decimate_trajectory_collision_index) +
                 trajectory_trim_index;
         i < (int)base_path.points.size(); ++i) {
      Eigen::Vector2d trajectory_vec;
      {
        const double yaw =
          getYawFromGeometryMsgsQuaternion(base_path.points.at(i).pose.orientation);
        trajectory_vec << std::cos(yaw), std::sin(yaw);
      }
      Eigen::Vector2d collision_point_vec;
      collision_point_vec << nearest_collision_point.x - base_path.points.at(i).pose.position.x,
        nearest_collision_point.y - base_path.points.at(i).pose.position.y;

      if (
        trajectory_vec.dot(collision_point_vec) < 0.0 ||
        (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(collision_point_vec))) {
        Eigen::Vector2d max_dist_stop_point;
        // search insert point
        size_t max_dist_stop_point_idx = 0;
        {
          double length_sum = 0.0;
          length_sum += trajectory_vec.normalized().dot(collision_point_vec);
          Eigen::Vector2d line_start_point, line_end_point;
          {
            line_start_point << base_path.points.at(0).pose.position.x,
              base_path.points.at(0).pose.position.y;
            const double yaw =
              getYawFromGeometryMsgsQuaternion(base_path.points.at(0).pose.orientation);
            line_end_point << std::cos(yaw), std::sin(yaw);
          }
          for (size_t j = i; 0 < j; --j) {
            line_start_point << base_path.points.at(j - 1).pose.position.x,
              base_path.points.at(j - 1).pose.position.y;
            line_end_point << base_path.points.at(j).pose.position.x,
              base_path.points.at(j).pose.position.y;
            if (stop_margin_ < length_sum) {
              max_dist_stop_point_idx = j;
              break;
            }
            length_sum += (line_end_point - line_start_point).norm();
          }
          getBackwordPointFromBasePoint(
            line_start_point, line_end_point, line_start_point, length_sum - stop_margin_,
            max_dist_stop_point);
        }
        Eigen::Vector2d min_dist_stop_point;
        size_t min_dist_stop_point_idx = 0;
        {
          double length_sum = 0.0;
          length_sum += trajectory_vec.normalized().dot(collision_point_vec);
          Eigen::Vector2d line_start_point, line_end_point;
          {
            line_start_point << base_path.points.at(0).pose.position.x,
              base_path.points.at(0).pose.position.y;
            const double yaw =
              getYawFromGeometryMsgsQuaternion(base_path.points.at(0).pose.orientation);
            line_end_point << std::cos(yaw), std::sin(yaw);
          }
          for (size_t j = i; 0 < j; --j) {
            line_start_point << base_path.points.at(j - 1).pose.position.x,
              base_path.points.at(j - 1).pose.position.y;
            line_end_point << base_path.points.at(j).pose.position.x,
              base_path.points.at(j).pose.position.y;
            if (min_behavior_stop_margin_ < length_sum) {
              min_dist_stop_point_idx = j;
              break;
            }
            length_sum += (line_end_point - line_start_point).norm();
          }
          getBackwordPointFromBasePoint(
            line_start_point, line_end_point, line_start_point,
            length_sum - min_behavior_stop_margin_, min_dist_stop_point);
        }

        // check already insert stop point
        bool is_inserted_already_stop_point = false;
        for (int j = max_dist_stop_point_idx - 1; j < (int)i; ++j) {
          if (base_path.points.at(std::max(j, 0)).twist.linear.x == 0.0) {
            is_inserted_already_stop_point = true;
            break;
          }
        }
        // insert stop point
        const size_t insert_stop_point_index =
          !is_inserted_already_stop_point ? max_dist_stop_point_idx : min_dist_stop_point_idx;
        const Eigen::Vector2d stop_point =
          !is_inserted_already_stop_point ? max_dist_stop_point : min_dist_stop_point;
        autoware_planning_msgs::msg::TrajectoryPoint stop_trajectory_point =
          base_path.points.at(std::max((int)(insert_stop_point_index)-1, 0));
        stop_trajectory_point.pose.position.x = stop_point.x();
        stop_trajectory_point.pose.position.y = stop_point.y();
        stop_trajectory_point.twist.linear.x = 0.0;
        output_msg.points.insert(
          output_msg.points.begin() + insert_stop_point_index, stop_trajectory_point);
        for (size_t j = insert_stop_point_index; j < output_msg.points.size(); ++j) {
          output_msg.points.at(j).twist.linear.x = 0.0;
        }

        stop_reason_diag = makeStopReasonDiag("obstacle", stop_trajectory_point.pose);
        debug_ptr_->pushPose(stop_trajectory_point.pose, PoseType::Stop);
        break;
      }
    }
  }

  /*
   * insert slow_down point
   */
  if (is_slow_down) {
    for (int i = decimate_trajectory_index_map.at(decimate_trajectory_slow_down_index);
         i < (int)base_path.points.size(); ++i) {
      Eigen::Vector2d trajectory_vec;
      {
        const double yaw =
          getYawFromGeometryMsgsQuaternion(base_path.points.at(i).pose.orientation);
        trajectory_vec << std::cos(yaw), std::sin(yaw);
      }
      Eigen::Vector2d slow_down_point_vec;
      slow_down_point_vec << nearest_slow_down_point.x - base_path.points.at(i).pose.position.x,
        nearest_slow_down_point.y - base_path.points.at(i).pose.position.y;

      if (
        trajectory_vec.dot(slow_down_point_vec) < 0.0 ||
        (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(slow_down_point_vec))) {
        Eigen::Vector2d slow_down_start_point;
        // search insert point
        size_t slow_down_point_idx = i;
        size_t slow_down_start_point_idx = 0;
        double slow_down_vel = 0.0;
        const double slow_down_target_vel =
          min_slow_down_vel_ + (max_slow_down_vel_ - min_slow_down_vel_) *
                                 std::max(lateral_deviation - vehicle_width_ / 2, 0.0) /
                                 expand_slow_down_range_;
        {
          double length_sum = 0.0;
          length_sum += trajectory_vec.normalized().dot(slow_down_point_vec);
          Eigen::Vector2d line_start_point, line_end_point;
          {
            line_start_point << base_path.points.at(0).pose.position.x,
              base_path.points.at(0).pose.position.y;
            const double yaw =
              getYawFromGeometryMsgsQuaternion(base_path.points.at(0).pose.orientation);
            line_end_point << std::cos(yaw), std::sin(yaw);
          }
          for (size_t j = i; 0 < j; --j) {
            line_start_point << base_path.points.at(j).pose.position.x,
              base_path.points.at(j).pose.position.y;
            line_end_point << base_path.points.at(j - 1).pose.position.x,
              base_path.points.at(j - 1).pose.position.y;
            if (slow_down_margin_ < length_sum) {
              slow_down_start_point_idx = j;
              break;
            }
            length_sum += (line_end_point - line_start_point).norm();
          }
          getBackwordPointFromBasePoint(
            line_start_point, line_end_point, line_start_point, length_sum - slow_down_margin_,
            slow_down_start_point);
          slow_down_vel = std::max(
            std::sqrt(
              slow_down_target_vel * slow_down_target_vel + 2 * max_deceleration_ * length_sum),
            current_velocity_ptr_->twist.linear.x);
        }

        autoware_planning_msgs::msg::TrajectoryPoint slow_down_start_trajectory_point =
          base_path.points.at(std::max((int)(slow_down_start_point_idx)-1, 0));
        autoware_planning_msgs::msg::TrajectoryPoint slow_down_end_trajectory_point;
        slow_down_start_trajectory_point.pose.position.x = slow_down_start_point.x();
        slow_down_start_trajectory_point.pose.position.y = slow_down_start_point.y();
        slow_down_start_trajectory_point.twist.linear.x = slow_down_vel;
        output_msg.points.insert(
          output_msg.points.begin() + slow_down_start_point_idx, slow_down_start_trajectory_point);
        bool is_slow_down_end = false;
        for (size_t j = slow_down_start_point_idx; j < output_msg.points.size() - 1; ++j) {
          output_msg.points.at(j).twist.linear.x =
            std::min(slow_down_vel, output_msg.points.at(j).twist.linear.x);
          const auto dist = std::hypot(
            output_msg.points.at(j).pose.position.x - output_msg.points.at(j + 1).pose.position.x,
            output_msg.points.at(j).pose.position.y - output_msg.points.at(j + 1).pose.position.y);
          slow_down_vel = std::max(
            slow_down_target_vel,
            std::sqrt(std::max(slow_down_vel * slow_down_vel - 2 * max_deceleration_ * dist, 0.0)));
          if (!is_slow_down_end && slow_down_vel <= slow_down_target_vel) {
            slow_down_end_trajectory_point = output_msg.points.at(j + 1);
            is_slow_down_end = true;
          }
        }
        if (!is_slow_down_end) {
          slow_down_end_trajectory_point = output_msg.points.back();
          is_slow_down_end = true;
        }
        debug_ptr_->pushPose(slow_down_start_trajectory_point.pose, PoseType::SlowDownStart);
        debug_ptr_->pushPose(slow_down_end_trajectory_point.pose, PoseType::SlowDownEnd);
      }
    }
  }
  path_pub_->publish(output_msg);
  stop_reason_diag_pub_->publish(stop_reason_diag);
  debug_ptr_->publish();
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

bool ObstacleStopPlannerNode::decimateTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length,
  autoware_planning_msgs::msg::Trajectory & output_trajectory)
{
  std::map<size_t /* decimate */, size_t /* origin */> index_map;
  decimateTrajectory(input_trajectory, step_length, output_trajectory, index_map);
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
  Eigen::Vector2d point1, point2;

  for (int i = 0; i < (int)(input_trajectory.points.size()) - 1; ++i) {
    if (next_length <= trajectory_length_sum + epsilon) {
      Eigen::Vector2d line_start_point, line_end_point, interporated_point;
      line_start_point << input_trajectory.points.at(i).pose.position.x,
        input_trajectory.points.at(i).pose.position.y;
      line_end_point << input_trajectory.points.at(i + 1).pose.position.x,
        input_trajectory.points.at(i + 1).pose.position.y;
      getBackwordPointFromBasePoint(
        line_start_point, line_end_point, line_end_point,
        -1.0 * (trajectory_length_sum - next_length), interporated_point);
      autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
      trajectory_point = input_trajectory.points.at(i);
      trajectory_point.pose.position.x = interporated_point.x();
      trajectory_point.pose.position.y = interporated_point.y();
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
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const geometry_msgs::msg::Pose self_pose,
  autoware_planning_msgs::msg::Trajectory & output_trajectory, size_t & index)
{
  double min_distance = 0.0;
  size_t min_dintance_index = 0;
  bool is_init = false;
  for (size_t i = 0; i < input_trajectory.points.size(); ++i) {
    const double x = input_trajectory.points.at(i).pose.position.x - self_pose.position.x;
    const double y = input_trajectory.points.at(i).pose.position.y - self_pose.position.y;
    const double squared_distance = x * x + y * y;
    if (!is_init || squared_distance < min_distance * min_distance) {
      is_init = true;
      min_distance = std::sqrt(squared_distance);
      min_dintance_index = i;
    }
  }
  for (size_t i = min_dintance_index; i < input_trajectory.points.size(); ++i) {
    output_trajectory.points.push_back(input_trajectory.points.at(i));
  }
  output_trajectory.header = input_trajectory.header;
  index = min_dintance_index;
  return true;
}

bool ObstacleStopPlannerNode::trimTrajectoryFromSelfPose(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const geometry_msgs::msg::Pose self_pose,
  autoware_planning_msgs::msg::Trajectory & output_trajectory)
{
  size_t index;
  return trimTrajectoryWithIndexFromSelfPose(
    output_trajectory, self_pose, output_trajectory, index);
}

bool ObstacleStopPlannerNode::searchPointcloudNearTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double radius,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr)
{
  const double squared_radius = radius * radius;
  for (const auto & trajectory_point : trajectory.points) {
    const auto center_pose = getVehicleCenterFromBase(trajectory_point.pose);
    for (const auto & point : input_pointcloud_ptr->points) {
      const double x = center_pose.position.x - point.x;
      const double y = center_pose.position.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) output_pointcloud_ptr->points.push_back(point);
    }
  }
  return true;
}

void ObstacleStopPlannerNode::createOneStepPolygon(
  const geometry_msgs::msg::Pose base_stap_pose, const geometry_msgs::msg::Pose next_step_pose,
  std::vector<cv::Point2d> & polygon, const double expand_width)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;
  // start step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(base_stap_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
      base_stap_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
      base_stap_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
      base_stap_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
      base_stap_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
  }
  // next step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(next_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
      next_step_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
      next_step_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0 - expand_width),
      next_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0 + expand_width),
      next_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0 + expand_width)));
  }
  convexHull(one_step_move_vehicle_corner_points, polygon);

  return;
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
  centroid.x = centroid.x / (double)pointcloud.size();
  centroid.y = centroid.y / (double)pointcloud.size();

  std::vector<cv::Point> normalized_pointcloud;
  std::vector<cv::Point> normalized_polygon_points;
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    normalized_pointcloud.push_back(cv::Point(
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
      tf_buffer.lookupTransform(header.frame_id, "base_link", header.stamp, rclcpp::Duration::from_seconds(0.1));
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
bool ObstacleStopPlannerNode::getBackwordPointFromBasePoint(
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
  Eigen::Vector2d base_pose_vec;
  base_pose_vec << std::cos(yaw), std::sin(yaw);

  for (size_t i = 0; i < pointcloud.size(); ++i) {
    Eigen::Vector2d pointcloud_vec;
    pointcloud_vec << pointcloud.at(i).x - base_pose.position.x,
      pointcloud.at(i).y - base_pose.position.y;
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
  Eigen::Vector2d base_pose_vec;
  base_pose_vec << std::cos(yaw), std::sin(yaw);
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    Eigen::Vector2d pointcloud_vec;
    pointcloud_vec << pointcloud.at(i).x - base_pose.position.x,
      pointcloud.at(i).y - base_pose.position.y;
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
