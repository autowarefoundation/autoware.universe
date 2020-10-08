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

#include <dynamic_reconfigure/server.h>
#include <boost/optional/optional_fwd.hpp>
#include <mutex>
#include "obstacle_avoidance_planner/AvoidancePlannerConfig.h"

#include <nav_msgs/MapMetaData.h>

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
struct VehicleParam;
struct MPTParam;
struct DebugData;
struct Trajectories;

class ObstacleAvoidancePlanner
{
private:
  bool is_publishing_clearance_map_;
  bool is_publishing_area_with_objects_;
  bool is_showing_debug_info_;
  bool is_using_vehicle_config_;
  bool is_stopping_if_outside_drivable_area_;
  bool enable_avoidance_;
  const int min_num_points_for_getting_yaw_;
  std::mutex mutex_;

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
  std::unique_ptr<VehicleParam> vehicle_param_;
  std::unique_ptr<MPTParam> mpt_param_;

  std::unique_ptr<geometry_msgs::Pose> current_ego_pose_ptr_;
  std::unique_ptr<geometry_msgs::TwistStamped> current_twist_ptr_;
  std::unique_ptr<geometry_msgs::Pose> prev_ego_pose_ptr_;
  std::unique_ptr<Trajectories> prev_trajectories_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> prev_path_points_ptr_;
  std::unique_ptr<autoware_perception_msgs::DynamicObjectArray> in_objects_ptr_;

  dynamic_reconfigure::Server<obstacle_avoidance_planner::AvoidancePlannerConfig>
    dynamic_reconfigure_srv_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::unique_ptr<ros::Time> prev_replanned_time_ptr_;

  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Publisher trajectory_pub_;
  ros::Publisher avoiding_traj_pub_;
  ros::Publisher debug_smoothed_points_pub_;
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
  void configCallback(
    const obstacle_avoidance_planner::AvoidancePlannerConfig & config, const uint32_t level);

  void initialize();

  //generate fine trajectory
  std::vector<autoware_planning_msgs::TrajectoryPoint> generatePostProcessedTrajectory(
    const geometry_msgs::Pose & ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & merged_optimized_points) const;

  bool needReplan(
    const geometry_msgs::Pose & ego_pose,
    const std::unique_ptr<geometry_msgs::Pose> & prev_ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::unique_ptr<ros::Time> & previous_replanned_time,
    const std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> & prev_path_points,
    std::unique_ptr<Trajectories> & prev_traj_points);

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
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const;

  void publishingDebugData(
    const DebugData & debug_data, const autoware_planning_msgs::Path & path,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points);

  int calculateNonDecelerationRange(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points,
    const geometry_msgs::Pose & ego_pose, const geometry_msgs::Twist & ego_twist) const;

  Trajectories getTrajectoryInsideArea(
    const Trajectories & trajs, const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const cv::Mat & road_clearance_map, const nav_msgs::MapMetaData & map_info,
    DebugData * debug_data) const;

  boost::optional<Trajectories> calcTrajectoryInsideArea(
    const Trajectories & trajs, const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const cv::Mat & road_clearance_map, const nav_msgs::MapMetaData & map_info,
    DebugData * debug_data, const bool is_prev_traj = false) const;

  Trajectories getPrevTrajs(
    const std::vector<autoware_planning_msgs::PathPoint> & path_points) const;

  std::vector<autoware_planning_msgs::TrajectoryPoint> getPrevTrajectory(
    const std::vector<autoware_planning_msgs::PathPoint> & path_points) const;

  Trajectories makePrevTrajectories(
    const geometry_msgs::Pose & ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const Trajectories & trajs) const;

  Trajectories getBaseTrajectory(
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const Trajectories & current_trajs) const;

  boost::optional<int> getStopIdx(
    const std::vector<autoware_planning_msgs::PathPoint> & path_points, const Trajectories & trajs,
    const nav_msgs::MapMetaData & map_info, const cv::Mat & road_clearance_map,
    DebugData * debug_data) const;

public:
  ObstacleAvoidancePlanner();
  ~ObstacleAvoidancePlanner();
};

#endif
