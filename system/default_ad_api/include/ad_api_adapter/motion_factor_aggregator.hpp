// Copyright 2020 TIER IV, Inc.
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

#ifndef AD_API_ADAPTER__MOTION_FACTOR_AGGREGATOR_HPP_
#define AD_API_ADAPTER__MOTION_FACTOR_AGGREGATOR_HPP_

#include "autoware_ad_api_msgs/msg/motion_factor_array.hpp"

#include "ad_api_adapter/awapi_autoware_util.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware_api
{
class MotionFactorAggregator : public rclcpp::Node
{
public:
  explicit MotionFactorAggregator(rclcpp::NodeOptions & node_options);

private:
  void callbackSceneModuleMotionFactor(
    const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr msg_ptr);
  void callbackObstacleStopMotionFactor(
    const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr msg_ptr);
  void callbackSurroundObstacleMotionFactor(
    const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr msg_ptr);
  void callbackAutowareTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr);
  void callbackTimer();

  autoware_ad_api_msgs::msg::MotionFactorArray makeMotionFactorArray();
  void applyUpdate(const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr);
  bool checkMatchingReason(
    const autoware_ad_api_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_motion_factor_array,
    const autoware_ad_api_msgs::msg::MotionFactorArray & motion_factor_array);
  void applyTimeOut();
  void appendMotionFactorToArray(
    const autoware_ad_api_msgs::msg::MotionFactor & motion_factor,
    autoware_ad_api_msgs::msg::MotionFactorArray * motion_factor_array);
  autoware_ad_api_msgs::msg::MotionFactor inputStopDistToMotionFactor(
    const autoware_ad_api_msgs::msg::MotionFactor & motion_factor);
  void getCurrentPose();

  rclcpp::Publisher<autoware_ad_api_msgs::msg::MotionFactorArray>::SharedPtr
    pub_motion_factor_;
  rclcpp::Subscription<autoware_ad_api_msgs::msg::MotionFactorArray>::SharedPtr
    sub_scene_module_motion_factor_;
  rclcpp::Subscription<autoware_ad_api_msgs::msg::MotionFactorArray>::SharedPtr
    sub_obstacle_stop_motion_factor_;
  rclcpp::Subscription<autoware_ad_api_msgs::msg::MotionFactorArray>::SharedPtr
    sub_surround_obstacle_motion_factor_;
  
  // rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::TimerBase::SharedPtr timer_;
  double timeout_;
  double status_pub_hz_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  

  std::vector<autoware_ad_api_msgs::msg::MotionFactorArray> motion_factor_array_vec_;
  autoware_ad_api_msgs::msg::MotionFactorArray obstacle_stop_factor_;
  autoware_ad_api_msgs::msg::MotionFactorArray surround_obstacle_factor_;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr autoware_planning_traj_ptr_;

  std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_ptr_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_MOTION_FACTOR_AGGREGATOR_HPP_
