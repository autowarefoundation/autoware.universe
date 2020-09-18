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
#pragma once
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace motion_planning
{
enum class PolygonType : int8_t { Vehicle = 0, Collision, SlowDownRange, SlowDown };

enum class PointType : int8_t { Stop = 0, SlowDown };

enum class PoseType : int8_t { Stop = 0, SlowDownStart, SlowDownEnd };

class ObstacleStopPlannerDebugNode
{
public:
  ObstacleStopPlannerDebugNode(const double base_link2front);
  ~ObstacleStopPlannerDebugNode(){};
  bool pushPolygon(
    const std::vector<cv::Point2d> & polygon, const double z, const PolygonType & type);
  bool pushPolygon(const std::vector<Eigen::Vector3d> & polygon, const PolygonType & type);
  bool pushPose(const geometry_msgs::Pose & pose, const PoseType & type);
  bool pushObstaclePoint(const geometry_msgs::Point & obstacle_point, const PointType & type);
  bool pushObstaclePoint(const pcl::PointXYZ & obstacle_point, const PointType & type);

  void publish();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;
  double base_link2front_;

  std::shared_ptr<geometry_msgs::Pose> stop_pose_ptr_;
  std::shared_ptr<geometry_msgs::Pose> slow_down_start_pose_ptr_;
  std::shared_ptr<geometry_msgs::Pose> slow_down_end_pose_ptr_;
  std::shared_ptr<geometry_msgs::Point> stop_obstacle_point_ptr_;
  std::shared_ptr<geometry_msgs::Point> slow_down_obstacle_point_ptr_;
  std::vector<std::vector<Eigen::Vector3d>> vehicle_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> slow_down_range_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> collision_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> slow_down_polygons_;
};

}  // namespace motion_planning
