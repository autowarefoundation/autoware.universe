/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "kalman_filter/kalman_filter.hpp"
#include "kalman_filter/time_delay_kalman_filter.hpp"

class EKFLocalizer
{
public:
  EKFLocalizer();
  ~EKFLocalizer();

private:
  ros::NodeHandle nh_;                  //!< @brief ros public node handler
  ros::NodeHandle pnh_;                 //!< @brief  ros private node handler
  ros::Publisher pub_pose_;             //!< @brief ekf estimated pose publisher
  ros::Publisher pub_pose_cov_;         //!< @brief estimated ekf pose with covariance publisher
  ros::Publisher pub_twist_;            //!< @brief ekf estimated twist publisher
  ros::Publisher pub_twist_cov_;        //!< @brief ekf estimated twist with covariance publisher
  ros::Publisher pub_debug_;            //!< @brief debug info publisher
  ros::Publisher pub_measured_pose_;    //!< @brief debug measurement pose publisher
  ros::Publisher pub_yaw_bias_;         //!< @brief ekf estimated yaw bias publisher
  ros::Publisher pub_pose_no_yawbias_;  //!< @brief ekf estimated yaw bias publisher
  ros::Publisher pub_pose_cov_no_yawbias_;  //!< @brief ekf estimated yaw bias publisher
  ros::Subscriber sub_initialpose_;         //!< @brief initial pose subscriber
  ros::Subscriber sub_pose_;                //!< @brief measurement pose subscriber
  ros::Subscriber sub_twist_;               //!< @brief measurement twist subscriber
  ros::Subscriber sub_pose_with_cov_;       //!< @brief measurement pose with covariance subscriber
  ros::Subscriber sub_twist_with_cov_;      //!< @brief measurement twist with covariance subscriber
  ros::Timer timer_control_;                //!< @brief time for ekf calculation callback
  ros::Timer timer_tf_;                     //!< @brief timer to send transform
  tf2_ros::TransformBroadcaster tf_br_;     //!< @brief tf broadcaster

  TimeDelayKalmanFilter ekf_;  //!< @brief  extended kalman filter instance.

  /* parameters */
  bool show_debug_info_;
  double ekf_rate_;  //!< @brief  EKF predict rate
  double ekf_dt_;    //!< @brief  = 1 / ekf_rate_
  double tf_rate_;   //!< @brief  tf publish rate
  bool
    enable_yaw_bias_estimation_;  //!< @brief  for LiDAR mount error. if true, publish /estimate_yaw_bias
  std::string pose_frame_id_;

  int dim_x_;              //!< @brief  dimension of EKF state
  int extend_state_step_;  //!< @brief  for time delay compensation
  int dim_x_ex_;  //!< @brief  dimension of extended EKF state (dim_x_ * extended_state_step)

  /* Pose */
  double
    pose_additional_delay_;  //!< @brief  compensated pose delay time = (pose.header.stamp - now) +
                             //!< additional_delay [s]
  double pose_measure_uncertainty_time_;  //!< @brief  added for measurement covariance
  double pose_rate_;  //!< @brief  pose rate [s], used for covariance calculation
  double
    pose_gate_dist_;      //!< @brief  the maharanobis distance threshold to ignore pose measurement
  double pose_stddev_x_;  //!< @brief  standard deviation for pose position x [m]
  double pose_stddev_y_;  //!< @brief  standard deviation for pose position y [m]
  double pose_stddev_yaw_;          //!< @brief  standard deviation for pose position yaw [rad]
  bool use_pose_with_covariance_;   //!< @brief  use covariance in pose_with_covarianve message
  bool use_twist_with_covariance_;  //!< @brief  use covariance in twist_with_covarianve message

  /* twist */
  double
    twist_additional_delay_;  //!< @brief  compensated delay = (twist.header.stamp - now) + additional_delay [s]
  double twist_rate_;  //!< @brief  rate [s], used for covariance calculation
  double
    twist_gate_dist_;  //!< @brief  measurement is ignored if the maharanobis distance is larger than this value.
  double twist_stddev_vx_;  //!< @brief  standard deviation for linear vx
  double twist_stddev_wz_;  //!< @brief  standard deviation for angular wx

  /* process noise variance for discrete model */
  double proc_cov_yaw_d_;       //!< @brief  discrete yaw process noise
  double proc_cov_yaw_bias_d_;  //!< @brief  discrete yaw bias process noise
  double proc_cov_vx_d_;        //!< @brief  discrete process noise in d_vx=0
  double proc_cov_wz_d_;        //!< @brief  discrete process noise in d_wz=0

  enum IDX {
    X = 0,
    Y = 1,
    YAW = 2,
    YAWB = 3,
    VX = 4,
    WZ = 5,
  };

  /* for model prediction */
  std::shared_ptr<geometry_msgs::TwistStamped>
    current_twist_ptr_;                                           //!< @brief current measured twist
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;  //!< @brief current measured pose
  geometry_msgs::PoseStamped current_ekf_pose_;                   //!< @brief current estimated pose
  geometry_msgs::PoseStamped
    current_ekf_pose_no_yawbias_;                  //!< @brief current estimated pose w/o yaw bias
  geometry_msgs::TwistStamped current_ekf_twist_;  //!< @brief current estimated twist
  boost::array<double, 36ul> current_pose_covariance_;
  boost::array<double, 36ul> current_twist_covariance_;

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timerCallback(const ros::TimerEvent & e);

  /**
   * @brief publish tf for tf_rate [Hz]
   */
  void timerTFCallback(const ros::TimerEvent & e);

  /**
   * @brief set pose measurement
   */
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr & msg);

  /**
   * @brief set twist measurement
   */
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg);

  /**
   * @brief set twistWithCovariance measurement
   */
  void callbackTwistWithCovariance(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr & msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped & msg);

  /**
   * @brief initialization of EKF
   */
  void initEKF();

  /**
   * @brief compute EKF prediction
   */
  void predictKinematicsModel();

  /**
   * @brief compute EKF update with pose measurement
   * @param pose measurement value
   */
  void measurementUpdatePose(const geometry_msgs::PoseStamped & pose);

  /**
   * @brief compute EKF update with pose measurement
   * @param twist measurement value
   */
  void measurementUpdateTwist(const geometry_msgs::TwistStamped & twist);

  /**
   * @brief check whether a measurement value falls within the mahalanobis distance threshold
   * @param dist_max mahalanobis distance threshold
   * @param estimated current estimated state
   * @param measured measured state
   * @param estimated_cov current estimation covariance
   * @return whether it falls within the mahalanobis distance threshold
   */
  bool mahalanobisGate(
    const double & dist_max, const Eigen::MatrixXd & estimated, const Eigen::MatrixXd & measured,
    const Eigen::MatrixXd & estimated_cov) const;

  /**
   * @brief get transform from frame_id
   */
  bool getTransformFromTF(
    std::string parent_frame, std::string child_frame, geometry_msgs::TransformStamped & transform);

  /**
   * @brief normalize yaw angle
   * @param yaw yaw angle
   * @return normalized yaw
   */
  double normalizeYaw(const double & yaw) const;

  /**
   * @brief create quaternion from roll, pitch and yaw.
   */
  geometry_msgs::Quaternion createQuaternionFromRPY(double r, double p, double y) const;

  /**
   * @brief set current EKF estimation result to current_ekf_pose_ & current_ekf_twist_
   */
  void setCurrentResult();

  /**
   * @brief publish current EKF estimation result
   */
  void publishEstimateResult();

  /**
   * @brief for debug
   */
  void showCurrentX();

  friend class EKFLocalizerTestSuite;  // for test code
};
