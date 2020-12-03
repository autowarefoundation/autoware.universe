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
#pragma once
#include <autoware_planning_msgs/msg/stop_reason_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
namespace motion_planning
{
enum class PolygonType : int8_t { Vehicle = 0, Collision, SlowDownRange, SlowDown };

enum class PointType : int8_t { Stop = 0, SlowDown };

enum class PoseType : int8_t { Stop = 0, SlowDownStart, SlowDownEnd };

class ObstacleStopPlannerDebugNode : public rclcpp::Node
{
public:
  ObstacleStopPlannerDebugNode(const double base_link2front);
  ~ObstacleStopPlannerDebugNode(){};
  bool pushPolygon(
    const std::vector<cv::Point2d> & polygon, const double z, const PolygonType & type);
  bool pushPolygon(const std::vector<Eigen::Vector3d> & polygon, const PolygonType & type);
  bool pushPose(const geometry_msgs::msg::Pose & pose, const PoseType & type);
  bool pushObstaclePoint(const geometry_msgs::msg::Point & obstacle_point, const PointType & type);
  bool pushObstaclePoint(const pcl::PointXYZ & obstacle_point, const PointType & type);
  visualization_msgs::msg::MarkerArray makeVisualizationMarker();
  autoware_planning_msgs::msg::StopReasonArray makeStopReasonArray();

  void publish();

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::StopReasonArray>::SharedPtr stop_reason_pub_;
  double base_link2front_;

  std::shared_ptr<geometry_msgs::msg::Pose> stop_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> slow_down_start_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> slow_down_end_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Point> stop_obstacle_point_ptr_;
  std::shared_ptr<geometry_msgs::msg::Point> slow_down_obstacle_point_ptr_;
  std::vector<std::vector<Eigen::Vector3d>> vehicle_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> slow_down_range_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> collision_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> slow_down_polygons_;
};

}  // namespace motion_planning
