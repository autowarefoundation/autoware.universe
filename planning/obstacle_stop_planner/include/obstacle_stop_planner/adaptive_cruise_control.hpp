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

#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/utils.h>

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Trajectory.h>

namespace motion_planning
{
class AdaptiveCruiseController
{
public:
  AdaptiveCruiseController(
    const double vehicle_width, const double vehicle_length, const double wheel_base,
    const double front_overhang);

  void insertAdaptiveCruiseVelocity(
    const autoware_planning_msgs::Trajectory & trajectory, const int nearest_collision_point_idx,
    const geometry_msgs::Pose self_pose, const pcl::PointXYZ & nearest_collision_point,
    const ros::Time nearest_collision_point_time,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr object_ptr,
    const geometry_msgs::TwistStamped::ConstPtr current_velocity_ptr, bool * need_to_stop,
    autoware_planning_msgs::Trajectory * output_trajectory);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_debug_;

  /*
   * Parameter
   */
  double vehicle_width_;
  double vehicle_length_;
  double wheel_base_;
  double front_overhang_;

  ros::Time prev_collsion_point_time_;
  pcl::PointXYZ prev_collsion_point_;
  double prev_target_vehicle_time_ = 0.0;
  double prev_target_vehicle_dist_ = 0.0;
  bool prev_collsion_point_valid_ = false;
  std::vector<geometry_msgs::TwistStamped> est_vel_que_;
  double prev_upper_velocity_ = 0.0;

  struct Param
  {
    double stop_margin;
    double min_behavior_stop_margin;

    //!< @brief use tracking objects for estimating object velocity or not
    bool use_object_to_est_vel;

    //!< @brief use pcl for estimating object velocity or not
    bool use_pcl_to_est_vel;

    //!< @brief consider forward vehicle velocity to self upper velocity or not
    bool consider_obj_velocity;

    //!< @brief The distance to extend the polygon length the object in pointcloud-object matching
    double object_polygon_length_margin;

    //!< @brief The distance to extend the polygon width the object in pointcloud-object matching
    double object_polygon_width_margin;

    //!< @breif Maximum time difference treated as continuous points in speed estimation using a point cloud
    double valid_est_vel_diff_time;

    //!< @brief Time width of information used for speed estimation in speed estimation using a point cloud
    double valid_vel_que_time;

    //!< @brief Maximum value of valid speed estimation results in speed estimation using a point cloud
    double valid_est_vel_max;

    //!< @brief Minimum value of valid speed estimation results in speed estimation using a point cloud
    double valid_est_vel_min;

    //!< @brief Embed a stop line if the maximum speed calculated by ACC is lowar than this speed
    double thresh_vel_to_stop;

    /* parameter for acc */
    //!< @brief threshold of forward obstacle velocity to insert stop line (to stop acc)
    double obstacle_stop_velocity_thresh;

    //!< @brief supposed minimum acceleration in emergency stop
    double emergency_stop_acceleration;

    //!< @brief supposed idling time to start emergency stop
    double emergency_stop_idling_time;

    //!< @brief minimum distance of emergency stop
    double min_dist_stop;

    //!< @brief supposed maximum acceleration in active cruise control
    double max_standard_acceleration;

    //!< @brief supposed minimum acceleration(deceleration) in active cruise control
    double min_standard_acceleration;

    //!< @brief supposed idling time to react object in active cruise control
    double standard_idling_time;

    //!< @brief minimum distance in active cruise control
    double min_dist_standard;

    //!< @brief supposed minimum acceleration of forward obstacle
    double obstacle_min_standard_acceleration;

    //!< @brief margin to insert upper velocity
    double margin_rate_to_change_vel;

    //!< @brief gain of lowpass filter of upper velocity
    double lowpass_gain_;

    /* parameter for pid used in acc */
    //!< @brief coefficient P in PID control (used when target dist -current_dist >=0)
    double p_coeff_pos;

    //!< @brief coefficient P in PID control (used when target dist -current_dist <0)
    double p_coeff_neg;

    //!< @brief coefficient D in PID control (used when delta_dist >=0)
    double d_coeff_pos;

    //!< @brief coefficient D in PID control (used when delta_dist <0)
    double d_coeff_neg;

    static constexpr double d_coeff_valid_time = 1.0;
    static constexpr double d_coeff_valid_diff_vel = 20.0;
    static constexpr double d_max_vel_norm = 3.0;
  };
  Param param_;

  double getMedianVel(const std::vector<geometry_msgs::TwistStamped> vel_que);
  double lowpass_filter(const double current_value, const double prev_value, const double gain);
  void calcDistanceToNearestPointOnPath(
    const autoware_planning_msgs::Trajectory & trajectory, const int nearest_point_idx,
    const geometry_msgs::Pose & self_pose, const pcl::PointXYZ & nearest_collision_point,
    double * distance);
  double calcTrajYaw(
    const autoware_planning_msgs::Trajectory & trajectory, const int collsion_point_idx);
  bool estimatePointVelocityFromObject(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr object_ptr, const double traj_yaw,
    const pcl::PointXYZ & nearest_collision_point, double * velocity);
  bool estimatePointVelocityFromPcl(
    const double traj_yaw, const pcl::PointXYZ & nearest_collision_point,
    const ros::Time & nearest_collision_point_time, double * velocity);
  double calcUpperVelocity(const double dist_to_col, const double obj_vel, const double self_vel);
  double calcThreshDistToForwardObstacle(const double current_vel, const double obj_vel);
  double calcBaseDistToForwardObstacle(const double current_vel, const double obj_vel);
  double calcTargetVelocity_P(const double target_dist, const double current_dist);
  double calcTargetVelocity_I(const double target_dist, const double current_dist);
  double calcTargetVelocity_D(const double target_dist, const double current_dist);
  double calcTargetVelocityByPID(
    const double current_vel, const double current_dist, const double obj_vel);

  void insertMaxVelocityToPath(
    const double current_vel, const double target_vel, const double dist_to_collsion_point,
    autoware_planning_msgs::Trajectory * output_trajectory);
  void registerQueToVelocity(const double vel, const ros::Time & vel_time);

  /* Debug */
  mutable std_msgs::Float32MultiArray debug_values_;
  enum DBGVAL {
    ESTIMATED_VEL_PCL = 0,
    ESTIMATED_VEL_OBJ = 1,
    ESTIMATED_VEL_FINAL = 2,
    FORWARD_OBJ_DISTANCE = 3,
    CURRENT_VEL = 4,
    UPPER_VEL_P = 5,
    UPPER_VEL_I = 6,
    UPPER_VEL_D = 7,
    UPPER_VEL_RAW = 8,
    UPPER_VEL = 9

  };
  static constexpr unsigned int num_debug_values_ = 10;
};

}  // namespace motion_planning
