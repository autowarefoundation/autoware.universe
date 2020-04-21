/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef CONTROL_LATLON_MUXER_INCLUDE_LATLON_MUXER_NODE_HPP_
#define CONTROL_LATLON_MUXER_INCLUDE_LATLON_MUXER_NODE_HPP_

#include <memory>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <ros/ros.h>

class LatLonMuxer
{
public:
  LatLonMuxer();
  ~LatLonMuxer() = default;

private:
  void latCtrlCmdCallback(const autoware_control_msgs::ControlCommandStamped::ConstPtr msg);
  void lonCtrlCmdCallback(const autoware_control_msgs::ControlCommandStamped::ConstPtr msg);
  void publishCmd();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher control_cmd_pub_;
  ros::Subscriber lat_control_cmd_sub_;
  ros::Subscriber lon_control_cmd_sub_;

  std::shared_ptr<autoware_control_msgs::ControlCommandStamped> lat_cmd_;
  std::shared_ptr<autoware_control_msgs::ControlCommandStamped> lon_cmd_;
};

#endif  // CONTROL_LATLON_MUXER_INCLUDE_LATLON_MUXER_NODE_HPP_
