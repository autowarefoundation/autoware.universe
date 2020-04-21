/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <pcl/filters/voxel_grid.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <obstacle_stop_planner/node.hpp>
#include <vector>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace
{
template <class T>
T getParam(const ros::NodeHandle & nh, const std::string & key, const T & default_value)
{
  T value;
  nh.param<T>(key, value, default_value);
  return value;
}

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}
double getYawFromGeometryMsgsQuaternion(const geometry_msgs::Quaternion & quat)
{
  tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

  return yaw;
}
}  // namespace

namespace motion_planning
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

ObstacleStopPlannerNode::ObstacleStopPlannerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  // Subscribers
  obstacle_pointcloud_sub_ = pnh_.subscribe(
    "input/pointcloud", 1, &ObstacleStopPlannerNode::obstaclePointcloudCallback, this);
  path_sub_ = pnh_.subscribe("input/trajectory", 1, &ObstacleStopPlannerNode::pathCallback, this);
  // Publishers
  path_pub_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);

  // Vehicle Parameters
  wheel_base_ = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");
  front_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/front_overhang");
  rear_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/rear_overhang");
  left_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/left_overhang");
  right_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/right_overhang");
  vehicle_width_ = waitForParam<double>(pnh_, "/vehicle_info/vehicle_width");
  // Parameters
  stop_margin_ = getParam<double>(pnh_, "stop_margin", 5.0);
  min_behavior_stop_margin_ = getParam<double>(pnh_, "min_behavior_stop_margin", 2.0);
  stop_margin_ += wheel_base_ + front_overhang_;
  min_behavior_stop_margin_ += wheel_base_ + front_overhang_;
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(wheel_base_ + front_overhang_);
}

void ObstacleStopPlannerNode::obstaclePointcloudCallback(
  const sensor_msgs::PointCloud2::ConstPtr & input_msg)
{
  obstacle_ros_pointcloud_ptr_ = boost::make_shared<sensor_msgs::PointCloud2>();
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
  const autoware_planning_msgs::Trajectory::ConstPtr & input_msg)
{
  if (obstacle_ros_pointcloud_ptr_ == nullptr) return;

  autoware_planning_msgs::Trajectory output_msg = *input_msg;
  const double epsilon = 0.00001;
  /*
   * trim trajectory from self pose
   */
  geometry_msgs::Pose self_pose;
  getSelfPose(input_msg->header, tf_buffer_, self_pose);
  autoware_planning_msgs::Trajectory trim_trajectory;
  size_t trajectory_trim_index;
  trimTrajectoryWithIndexFromSelfPose(
    *input_msg, self_pose, trim_trajectory, trajectory_trim_index);

  /*
   * decimate trajectory for calculation cost
   */
  autoware_planning_msgs::Trajectory decimate_trajectory;
  const double step_length = 1.0;
  std::map<size_t /* decimate */, size_t /* origin */> decimate_trajectory_index_map;
  decimateTrajectory(
    trim_trajectory, step_length, decimate_trajectory, decimate_trajectory_index_map);

  autoware_planning_msgs::Trajectory & trajectory = decimate_trajectory;

  /*
   * search candidate obstacle pointcloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  {
    // transform pointcloud
    geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
      trajectory.header.frame_id, obstacle_ros_pointcloud_ptr_->header.frame_id,
      obstacle_ros_pointcloud_ptr_->header.stamp, ros::Duration(0.5));
    Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    sensor_msgs::PointCloud2 transformed_obstacle_ros_pointcloud;
    pcl_ros::transformPointCloud(
      affine_matrix, *obstacle_ros_pointcloud_ptr_, transformed_obstacle_ros_pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_obstacle_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_obstacle_ros_pointcloud, *transformed_obstacle_pointcloud_ptr);
    // search obstacle candidate pointcloud to reduce calculation cost
    const double search_range =
      step_length + std::hypot((vehicle_width_ / 2.0), (wheel_base_ + front_overhang_));
    searchPointcloudNearTrajectory(
      trajectory, search_range, transformed_obstacle_pointcloud_ptr,
      obstacle_candidate_pointcloud_ptr);
  }

  /*
   * check collision
   */
  bool is_collision = false;
  size_t decimate_trajectory_collision_index;
  pcl::PointXYZ nearest_collision_point;
  for (int i = 0; i < (int)(trajectory.points.size()) - 1; ++i) {
    /*
     * create one step polygon
     */
    std::vector<cv::Point2d> one_step_move_vehicle_polygon;
    createOneStepPolygon(
      trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
      one_step_move_vehicle_polygon);
    debug_ptr_->pushPolygon(one_step_move_vehicle_polygon, trajectory.points.at(i).pose.position.z);

    /*
     * collision check obstacle pointcloud and polygon
     */
    // convert boost polygon
    Polygon boost_one_step_move_vehicle_polygon;
    for (const auto & point : one_step_move_vehicle_polygon) {
      boost_one_step_move_vehicle_polygon.outer().push_back(bg::make<Point>(point.x, point.y));
    }
    boost_one_step_move_vehicle_polygon.outer().push_back(bg::make<Point>(
      one_step_move_vehicle_polygon.front().x, one_step_move_vehicle_polygon.front().y));
    // check within polygon
    pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t j = 0; j < obstacle_candidate_pointcloud_ptr->size(); ++j) {
      Point point(
        obstacle_candidate_pointcloud_ptr->at(j).x, obstacle_candidate_pointcloud_ptr->at(j).y);
      if (bg::within(point, boost_one_step_move_vehicle_polygon)) {
        collision_pointcloud_ptr->push_back(obstacle_candidate_pointcloud_ptr->at(j));
        is_collision = true;
        debug_ptr_->pushCollisionPolygon(
          one_step_move_vehicle_polygon, trajectory.points.at(i).pose.position.z);
      }
    }

    /*
     * search nearest collision point by begining of path
     */
    if (is_collision) {
      getNearestPoint(
        *collision_pointcloud_ptr, trajectory.points.at(i).pose, nearest_collision_point);
      debug_ptr_->pushStopObstaclePoint(nearest_collision_point);
      decimate_trajectory_collision_index = i;
      break;
    }
  }

  /*
   * insert stop point
   */
  if (is_collision) {
    for (int i = decimate_trajectory_index_map.at(decimate_trajectory_collision_index);
         i < (int)output_msg.points.size(); ++i) {
      Eigen::Vector2d trajectory_vec;
      {
        const double yaw =
          getYawFromGeometryMsgsQuaternion(output_msg.points.at(i).pose.orientation);
        trajectory_vec << std::cos(yaw), std::sin(yaw);
      }
      Eigen::Vector2d collision_point_vec;
      collision_point_vec << nearest_collision_point.x - output_msg.points.at(i).pose.position.x,
        nearest_collision_point.y - output_msg.points.at(i).pose.position.y;

      if (
        trajectory_vec.dot(collision_point_vec) < 0.0 ||
        (i + 1 == output_msg.points.size() && 0.0 < trajectory_vec.dot(collision_point_vec))) {
        Eigen::Vector2d max_dist_stop_point;
        // search insert point
        size_t max_dist_stop_point_idx = 0;
        {
          double length_sum = 0.0;
          length_sum += trajectory_vec.normalized().dot(collision_point_vec);
          Eigen::Vector2d line_start_point, line_end_point;
          {
            line_start_point << output_msg.points.at(0).pose.position.x,
              output_msg.points.at(0).pose.position.y;
            const double yaw =
              getYawFromGeometryMsgsQuaternion(output_msg.points.at(0).pose.orientation);
            line_end_point << std::cos(yaw), std::sin(yaw);
          }
          for (size_t j = i; 0 < j; --j) {
            line_start_point << output_msg.points.at(j - 1).pose.position.x,
              output_msg.points.at(j - 1).pose.position.y;
            line_end_point << output_msg.points.at(j).pose.position.x,
              output_msg.points.at(j).pose.position.y;
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
            line_start_point << output_msg.points.at(0).pose.position.x,
              output_msg.points.at(0).pose.position.y;
            const double yaw =
              getYawFromGeometryMsgsQuaternion(output_msg.points.at(0).pose.orientation);
            line_end_point << std::cos(yaw), std::sin(yaw);
          }
          for (size_t j = i; 0 < j; --j) {
            line_start_point << output_msg.points.at(j - 1).pose.position.x,
              output_msg.points.at(j - 1).pose.position.y;
            line_end_point << output_msg.points.at(j).pose.position.x,
              output_msg.points.at(j).pose.position.y;
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
          if (output_msg.points.at(std::max(j, 0)).twist.linear.x == 0.0) {
            is_inserted_already_stop_point = true;
            break;
          }
        }
        // insert stop point
        const size_t insert_stop_point_index =
          !is_inserted_already_stop_point ? max_dist_stop_point_idx : min_dist_stop_point_idx;
        const Eigen::Vector2d stop_point =
          !is_inserted_already_stop_point ? max_dist_stop_point : min_dist_stop_point;
        autoware_planning_msgs::TrajectoryPoint stop_trajectory_point =
          output_msg.points.at(std::max((int)(insert_stop_point_index)-1, 0));
        stop_trajectory_point.pose.position.x = stop_point.x();
        stop_trajectory_point.pose.position.y = stop_point.y();
        stop_trajectory_point.twist.linear.x = 0.0;
        output_msg.points.insert(
          output_msg.points.begin() + insert_stop_point_index, stop_trajectory_point);
        for (size_t j = insert_stop_point_index; j < output_msg.points.size(); ++j) {
          output_msg.points.at(j).twist.linear.x = 0.0;
        }
        debug_ptr_->pushStopPose(stop_trajectory_point.pose);
        break;
      }
    }
  }
  path_pub_.publish(output_msg);
  debug_ptr_->publish();
}

bool ObstacleStopPlannerNode::decimateTrajectory(
  const autoware_planning_msgs::Trajectory & input_trajectory, const double step_length,
  autoware_planning_msgs::Trajectory & output_trajectory)
{
  std::map<size_t /* decimate */, size_t /* origin */> index_map;
  decimateTrajectory(input_trajectory, step_length, output_trajectory, index_map);
}

bool ObstacleStopPlannerNode::decimateTrajectory(
  const autoware_planning_msgs::Trajectory & input_trajectory, const double step_length,
  autoware_planning_msgs::Trajectory & output_trajectory,
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
      autoware_planning_msgs::TrajectoryPoint trajectory_point;
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
  const autoware_planning_msgs::Trajectory & input_trajectory, const geometry_msgs::Pose self_pose,
  autoware_planning_msgs::Trajectory & output_trajectory, size_t & index)
{
  double min_distance;
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
  const autoware_planning_msgs::Trajectory & input_trajectory, const geometry_msgs::Pose self_pose,
  autoware_planning_msgs::Trajectory & output_trajectory)
{
  size_t index;
  return trimTrajectoryWithIndexFromSelfPose(
    output_trajectory, self_pose, output_trajectory, index);
}

bool ObstacleStopPlannerNode::searchPointcloudNearTrajectory(
  const autoware_planning_msgs::Trajectory & trajectory, const double radius,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr)
{
  const double squared_radius = radius * radius;
  for (const auto & trajectory_point : trajectory.points) {
    for (const auto & point : input_pointcloud_ptr->points) {
      const double x = trajectory_point.pose.position.x - point.x;
      const double y = trajectory_point.pose.position.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) output_pointcloud_ptr->points.push_back(point);
    }
  }
  return true;
}

void ObstacleStopPlannerNode::createOneStepPolygon(
  const geometry_msgs::Pose base_stap_pose, const geometry_msgs::Pose next_step_pose,
  std::vector<cv::Point2d> & polygon)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;
  // start step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(base_stap_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0),
      base_stap_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0),
      base_stap_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0),
      base_stap_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_stap_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0),
      base_stap_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0)));
  }
  // next step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(next_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0),
      next_step_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (wheel_base_ + front_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0),
      next_step_pose.position.y + std::sin(yaw) * (wheel_base_ + front_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (-vehicle_width_ / 2.0),
      next_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (-vehicle_width_ / 2.0)));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * (-rear_overhang_) -
        std::sin(yaw) * (vehicle_width_ / 2.0),
      next_step_pose.position.y + std::sin(yaw) * (-rear_overhang_) +
        std::cos(yaw) * (vehicle_width_ / 2.0)));
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
  const std_msgs::Header & header, const tf2_ros::Buffer & tf_buffer,
  geometry_msgs::Pose & self_pose)
{
  try {
    geometry_msgs::TransformStamped transform;
    transform =
      tf_buffer.lookupTransform(header.frame_id, "base_link", header.stamp, ros::Duration(0.1));
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
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::Pose & base_pose,
  pcl::PointXYZ & nearest_collision_point)
{
  double min_norm;
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
      nearest_collision_point = pointcloud.at(i);
      is_init = true;
    }
  }
}

}  // namespace motion_planning