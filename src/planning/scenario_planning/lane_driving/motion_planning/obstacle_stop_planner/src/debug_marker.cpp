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
#include "obstacle_stop_planner/debug_marker.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// namespace {
// convertPose2Transform
// }
namespace motion_planning
{
ObstacleStopPlannerDebugNode::ObstacleStopPlannerDebugNode(const double base_link2front)
: nh_(), pnh_("~"), base_link2front_(base_link2front)
{
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/marker", 1);
}

void ObstacleStopPlannerDebugNode::pushPolygon(
  const std::vector<cv::Point2d> & polygon, const double z)
{
  std::vector<Eigen::Vector3d> eigen_polygon;
  for (const auto & point : polygon) {
    Eigen::Vector3d eigen_point;
    eigen_point << point.x, point.y, z;
    eigen_polygon.push_back(eigen_point);
  }
  pushPolygon(eigen_polygon);
}

void ObstacleStopPlannerDebugNode::pushPolygon(const std::vector<Eigen::Vector3d> & polygon)
{
  if (!polygon.empty()) polygons_.push_back(polygon);
}

void ObstacleStopPlannerDebugNode::pushCollisionPolygon(
  const std::vector<cv::Point2d> & polygon, const double z)
{
  std::vector<Eigen::Vector3d> eigen_polygon;
  for (const auto & point : polygon) {
    Eigen::Vector3d eigen_point;
    eigen_point << point.x, point.y, z;
    eigen_polygon.push_back(eigen_point);
  }
  pushCollisionPolygon(eigen_polygon);
}

void ObstacleStopPlannerDebugNode::pushCollisionPolygon(
  const std::vector<Eigen::Vector3d> & polygon)
{
  if (!polygon.empty()) collision_polygons_.push_back(polygon);
}

void ObstacleStopPlannerDebugNode::pushStopPose(const geometry_msgs::Pose & stop_pose)
{
  stop_pose_ptr_ = std::make_shared<geometry_msgs::Pose>(stop_pose);
}

void ObstacleStopPlannerDebugNode::pushStopObstaclePoint(
  const geometry_msgs::Point & stop_obstacle_point)
{
  stop_obstacle_point_ptr_ = std::make_shared<geometry_msgs::Point>(stop_obstacle_point);
}

void ObstacleStopPlannerDebugNode::pushStopObstaclePoint(const pcl::PointXYZ & stop_obstacle_point)
{
  geometry_msgs::Point ros_point;
  ros_point.x = stop_obstacle_point.x;
  ros_point.y = stop_obstacle_point.y;
  ros_point.z = stop_obstacle_point.z;
  pushStopObstaclePoint(ros_point);
}

void ObstacleStopPlannerDebugNode::publish()
{
  visualization_msgs::MarkerArray msg;
  ros::Time current_time = ros::Time::now();
  tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(base_link2front_, 0.0, 0.0));

  // polygon
  if (!polygons_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "detection_polygons";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t i = 0; i < polygons_.size(); ++i) {
      for (size_t j = 0; j < polygons_.at(i).size(); ++j) {
        {
          geometry_msgs::Point point;
          point.x = polygons_.at(i).at(j).x();
          point.y = polygons_.at(i).at(j).y();
          point.z = polygons_.at(i).at(j).z();
          marker.points.push_back(point);
        }
        if (j + 1 == polygons_.at(i).size()) {
          geometry_msgs::Point point;
          point.x = polygons_.at(i).at(0).x();
          point.y = polygons_.at(i).at(0).y();
          point.z = polygons_.at(i).at(0).z();
          marker.points.push_back(point);

        } else {
          geometry_msgs::Point point;
          point.x = polygons_.at(i).at(j + 1).x();
          point.y = polygons_.at(i).at(j + 1).y();
          point.z = polygons_.at(i).at(j + 1).z();
          marker.points.push_back(point);
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!collision_polygons_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "collision_polygons";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t i = 0; i < collision_polygons_.size(); ++i) {
      for (size_t j = 0; j < collision_polygons_.at(i).size(); ++j) {
        {
          geometry_msgs::Point point;
          point.x = collision_polygons_.at(i).at(j).x();
          point.y = collision_polygons_.at(i).at(j).y();
          point.z = collision_polygons_.at(i).at(j).z();
          marker.points.push_back(point);
        }
        if (j + 1 == collision_polygons_.at(i).size()) {
          geometry_msgs::Point point;
          point.x = collision_polygons_.at(i).at(0).x();
          point.y = collision_polygons_.at(i).at(0).y();
          point.z = collision_polygons_.at(i).at(0).z();
          marker.points.push_back(point);

        } else {
          geometry_msgs::Point point;
          point.x = collision_polygons_.at(i).at(j + 1).x();
          point.y = collision_polygons_.at(i).at(j + 1).y();
          point.z = collision_polygons_.at(i).at(j + 1).z();
          marker.points.push_back(point);
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (stop_pose_ptr_ != nullptr) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "stop_virtual_wall";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*stop_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }

  if (stop_pose_ptr_ != nullptr) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "factor_text";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*stop_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "obstacle";
    msg.markers.push_back(marker);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "stop_obstacle_point";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "stop_obstacle_text";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "!";
    msg.markers.push_back(marker);
  }

  debug_viz_pub_.publish(msg);
  polygons_.clear();
  collision_polygons_.clear();
  stop_pose_ptr_ = nullptr;
  stop_obstacle_point_ptr_ = nullptr;
  return;
}
}  // namespace motion_planning