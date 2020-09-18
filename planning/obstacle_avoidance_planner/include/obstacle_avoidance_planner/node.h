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
#ifndef NODE_H
#define NODE_H

#include <boost/optional/optional_fwd.hpp>

namespace ros
{
class Time;
}

namespace cv
{
class Mat;
}

namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace std_msgs
{
ROS_DECLARE_MESSAGE(Bool);
}

namespace autoware_planning_msgs
{
ROS_DECLARE_MESSAGE(PathPoint);
ROS_DECLARE_MESSAGE(Path);
ROS_DECLARE_MESSAGE(TrajectoryPoint);
ROS_DECLARE_MESSAGE(Trajectory);
}  // namespace autoware_planning_msgs

namespace autoware_perception_msgs
{
ROS_DECLARE_MESSAGE(DynamicObjectArray);
}  // namespace autoware_perception_msgs

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(Pose);
ROS_DECLARE_MESSAGE(Point);
ROS_DECLARE_MESSAGE(TwistStamped);
ROS_DECLARE_MESSAGE(Twist);
}  // namespace geometry_msgs

class EBPathOptimizer;

struct QPParam;
struct TrajectoryParam;
struct ConstrainParam;
struct DebugData;

class ObstacleAvoidancePlanner
{
private:
  bool is_publishing_clearance_map_;
  bool is_publishing_area_with_objects_;
  bool is_showing_debug_info_;
  bool is_using_vehicle_config_;
  bool enable_avoidance_;
  const int min_num_points_for_getting_yaw_;

  // params outside logic
  double min_delta_dist_for_replan_;
  double min_delta_time_sec_for_replan_;
  double max_dist_for_extending_end_point_;
  double distance_for_path_shape_chagne_detection_;

  // logic
  std::unique_ptr<EBPathOptimizer> eb_path_optimizer_ptr_;

  // params
  std::unique_ptr<QPParam> qp_param_;
  std::unique_ptr<TrajectoryParam> traj_param_;
  std::unique_ptr<ConstrainParam> constrain_param_;

  std::unique_ptr<geometry_msgs::Pose> current_ego_pose_ptr_;
  std::unique_ptr<geometry_msgs::TwistStamped> current_twist_ptr_;
  std::unique_ptr<geometry_msgs::Pose> prev_ego_pose_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> prev_optimized_points_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> prev_traj_points_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> prev_path_points_ptr_;
  std::unique_ptr<autoware_perception_msgs::DynamicObjectArray> in_objects_ptr_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::unique_ptr<ros::Time> prev_replanned_time_ptr_;

  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Publisher trajectory_pub_;
  ros::Publisher avoiding_traj_pub_;
  ros::Publisher is_avoidance_possible_pub_;
  ros::Publisher debug_markers_pub_;
  ros::Publisher debug_clearance_map_pub_;
  ros::Publisher debug_object_clearance_map_pub_;
  ros::Publisher debug_area_with_objects_pub_;
  ros::Subscriber path_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber is_avoidance_sub_;

  // callback functions
  void pathCallback(const autoware_planning_msgs::Path & msg);
  void twistCallback(const geometry_msgs::TwistStamped & msg);
  void objectsCallback(const autoware_perception_msgs ::DynamicObjectArray & msg);
  void enableAvoidanceCallback(const std_msgs::Bool & msg);

  void initialize();

  std::vector<autoware_planning_msgs::TrajectoryPoint> generatePostProcessedTrajectory(
    const geometry_msgs::Pose & ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & merged_optimized_points);

  bool needReplan(
    const geometry_msgs::Pose & ego_pose,
    const std::unique_ptr<geometry_msgs::Pose> & prev_ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::unique_ptr<ros::Time> & previous_replanned_time,
    const std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> & prev_path_points,
    std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> & prev_traj_points);

  std::vector<autoware_planning_msgs::TrajectoryPoint> generateOptimizedTrajectory(
    const geometry_msgs::Pose & ego_pose, const autoware_planning_msgs::Path & input_path);

  std::unique_ptr<geometry_msgs::Pose> getCurrentEgoPose();

  bool isPathShapeChanged(
    const geometry_msgs::Pose & ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> & prev_path_points);

  autoware_planning_msgs::Trajectory generateTrajectory(
    const autoware_planning_msgs::Path & in_path);

  std::vector<autoware_planning_msgs::TrajectoryPoint> convertPointsToTrajectory(
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points);

  boost::optional<geometry_msgs::Point> getExtendedPoint(
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::vector<geometry_msgs::Point> & interpolated_points);

  std::vector<autoware_planning_msgs::TrajectoryPoint> getOptimizedPointsWhenAborting(
    const std::vector<autoware_planning_msgs::PathPoint> & path_points);

  void publishingDebugData(
    const DebugData & debug_data, const autoware_planning_msgs::Path & path,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points);

  int calculateNonDecelerationRange(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points,
    const geometry_msgs::Pose & ego_pose, const geometry_msgs::Twist & ego_twist);

public:
  ObstacleAvoidancePlanner();
  ~ObstacleAvoidancePlanner();
};

#endif
