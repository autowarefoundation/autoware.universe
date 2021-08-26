// Copyright 2019 Autoware Foundation
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
#ifndef OBSTACLE_STOP_PLANNER__NODE_HPP_
#define OBSTACLE_STOP_PLANNER__NODE_HPP_

#include <map>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/expand_stop_range.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "obstacle_stop_planner/adaptive_cruise_control.hpp"
#include "obstacle_stop_planner/debug_marker.hpp"

namespace motion_planning
{
struct StopPoint
{
  size_t index;
  Eigen::Vector2d point;
};

struct SlowDownPoint
{
  size_t index;
  Eigen::Vector2d point;
  double velocity;
};

class ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  /*
   * ROS
   */
  // publisher and subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_velocity_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    dynamic_object_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::ExpandStopRange>::SharedPtr
    expand_stop_range_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;

  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /*
   * Parameter
   */
  std::unique_ptr<motion_planning::AdaptiveCruiseController> acc_controller_;
  sensor_msgs::msg::PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr_;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr object_ptr_;
  double rear_overhang_, left_overhang_, right_overhang_, vehicle_width_, vehicle_length_,
    baselink2front_;
  double stop_margin_;
  double slow_down_margin_;
  double min_behavior_stop_margin_;
  rclcpp::Time prev_col_point_time_;
  pcl::PointXYZ prev_col_point_;
  double expand_slow_down_range_;
  double expand_stop_range_;
  double max_slow_down_vel_;
  double min_slow_down_vel_;
  double max_deceleration_;
  bool enable_slow_down_;
  double extend_distance_;
  double step_length_;
  double stop_search_radius_;
  double slow_down_search_radius_;
  void obstaclePointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void pathCallback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg);
  void dynamicObjectCallback(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);
  void currentVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg);
  void externalExpandStopRangeCallback(
    const autoware_planning_msgs::msg::ExpandStopRange::ConstSharedPtr input_msg);

private:
  bool convexHull(
    const std::vector<cv::Point2d> pointcloud, std::vector<cv::Point2d> & polygon_points);
  bool decimateTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);
  bool decimateTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length,
    autoware_planning_msgs::msg::Trajectory & output_trajectory,
    std::map<size_t /* decimate */, size_t /* origin */> & index_map);
  bool trimTrajectoryFromSelfPose(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const geometry_msgs::msg::Pose self_pose,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);
  bool trimTrajectoryWithIndexFromSelfPose(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const geometry_msgs::msg::Pose self_pose,
    autoware_planning_msgs::msg::Trajectory & output_trajectory,
    size_t & index);
  bool searchPointcloudNearTrajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const double radius,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr);
  void createOneStepPolygon(
    const geometry_msgs::msg::Pose base_step_pose, const geometry_msgs::msg::Pose next_step_pose,
    std::vector<cv::Point2d> & polygon, const double expand_width = 0.0);
  bool getSelfPose(
    const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::Pose & self_pose);
  bool getBackwardPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);
  void getNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time);
  void getLateralNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * lateral_nearest_point, double * deviation);
  geometry_msgs::msg::Pose getVehicleCenterFromBase(const geometry_msgs::msg::Pose & base_pose);

  void insertStopPoint(
    const StopPoint & stop_point, const autoware_planning_msgs::msg::Trajectory & base_path,
    autoware_planning_msgs::msg::Trajectory & output_path,
    diagnostic_msgs::msg::DiagnosticStatus & stop_reason_diag);

  StopPoint searchInsertPoint(
    const int idx, const autoware_planning_msgs::msg::Trajectory & base_path,
    const Eigen::Vector2d & trajectory_vec, const Eigen::Vector2d & collision_point_vec);

  StopPoint createTargetPoint(
    const int idx, const double margin, const Eigen::Vector2d & trajectory_vec,
    const Eigen::Vector2d & collision_point_vec,
    const autoware_planning_msgs::msg::Trajectory & base_path);

  SlowDownPoint createSlowDownStartPoint(
    const int idx, const double margin, const double slow_down_target_vel,
    const Eigen::Vector2d & trajectory_vec, const Eigen::Vector2d & slow_down_point_vec,
    const autoware_planning_msgs::msg::Trajectory & base_path);

  void insertSlowDownStartPoint(
    const SlowDownPoint & slow_down_start_point,
    const autoware_planning_msgs::msg::Trajectory & base_path,
    autoware_planning_msgs::msg::Trajectory & output_path);

  void insertSlowDownVelocity(
    const size_t slow_down_start_point_idx, const double slow_down_target_vel, double slow_down_vel,
    autoware_planning_msgs::msg::Trajectory & output_path);

  double calcSlowDownTargetVel(const double lateral_deviation);
  bool extendTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const double extend_distance,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);

  autoware_planning_msgs::msg::TrajectoryPoint getExtendTrajectoryPoint(
    double extend_distance, const autoware_planning_msgs::msg::TrajectoryPoint & goal_point);
};
}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__NODE_HPP_
