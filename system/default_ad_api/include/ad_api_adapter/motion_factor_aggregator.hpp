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

#include "autoware_ad_api_msgs/motion/msg/motion_factor_array.hpp"

#include "ad_api_adapter/awapi_autoware_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware_api
{
class AutowareIvMotionFactorAggregator
{
public:
  AutowareIvMotionFactorAggregator(rclcpp::Node & node, const double timeout);
  void updateSceneModuleMotionFactorArray(
    const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr);
  void updateObstacleStopMotionFactorArray(
    const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr);
  void updateSurroundObstacleMotionFactorArray(
    const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr);
  tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr makeMotionFactorArray(
    const AutowareInfo & aw_info);

private:
  void applyUpdate(const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_ptr);
  bool checkMatchingReason(
    const tier4_planning_msgs::msg::MotionFactorArray::ConstSharedPtr & msg_motion_factor_array,
    const tier4_planning_msgs::msg::MotionFactorArray & motion_factor_array);
  void applyTimeOut();
  void appendMotionFactorToArray(
    const tier4_planning_msgs::msg::MotionFactor & motion_factor,
    tier4_planning_msgs::msg::MotionFactorArray * motion_factor_array,
    const AutowareInfo & aw_info);
  tier4_planning_msgs::msg::MotionFactor inputStopDistToMotionFactor(
    const tier4_planning_msgs::msg::MotionFactor & motion_factor, const AutowareInfo & aw_info);
  double calcStopDistToStopFactor(
    const tier4_planning_msgs::msg::StopFactor & stop_factor, const AutowareInfo & aw_info);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double timeout_;

  std::vector<tier4_planning_msgs::msg::MotionFactorArray> motion_factor_array_vec_;
  tier4_planning_msgs::msg::MotionFactorArray obstacle_stop_factor_;
  tier4_planning_msgs::msg::MotionFactorArray surround_obstacle_factor_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_MOTION_FACTOR_AGGREGATOR_HPP_
