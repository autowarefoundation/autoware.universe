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

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <topic_tools/shape_shifter.h>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_system_msgs/AutowareState.h>
#include <autoware_vehicle_msgs/ControlMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

#include <autoware_state_monitor/autoware_state.h>
#include <autoware_state_monitor/config.h>
#include <autoware_state_monitor/state_machine.h>

class AutowareStateMonitorNode
{
public:
  AutowareStateMonitorNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  bool disengage_on_route_;
  bool disengage_on_complete_;
  bool disengage_on_emergency_;

  std::vector<TopicConfig> topic_configs_;
  std::vector<ParamConfig> param_configs_;
  std::vector<TfConfig> tf_configs_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  geometry_msgs::PoseStamped::ConstPtr current_pose_;

  // Subscriber
  ros::Subscriber sub_autoware_engage_;
  ros::Subscriber sub_vehicle_control_mode_;
  ros::Subscriber sub_is_emergency_;
  ros::Subscriber sub_route_;
  ros::Subscriber sub_twist_;

  void onAutowareEngage(const std_msgs::Bool::ConstPtr & msg);
  void onVehicleControlMode(const autoware_vehicle_msgs::ControlMode::ConstPtr & msg);
  void onIsEmergency(const std_msgs::Bool::ConstPtr & msg);
  void onRoute(const autoware_planning_msgs::Route::ConstPtr & msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);

  // Topic Buffer
  void onTopic(const topic_tools::ShapeShifter::ConstPtr & msg, const std::string & topic_name);
  void registerTopicCallback(const std::string & topic_name);

  std::map<std::string, ros::Subscriber> sub_topic_map_;
  std::map<std::string, std::deque<ros::Time>> topic_received_time_buffer_;

  // Publisher
  ros::Publisher pub_autoware_state_;
  ros::Publisher pub_autoware_engage_;

  void setDisengage();

  // Timer
  void onTimer(const ros::TimerEvent & event);
  ros::Timer timer_;

  // Stats
  TopicStats getTopicStats() const;
  ParamStats getParamStats() const;
  TfStats getTfStats() const;

  // State Machine
  std::shared_ptr<StateMachine> state_machine_;
  StateInput state_input_;
  StateParam state_param_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void setupDiagnosticUpdater();
  void checkTopicStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name);
  void checkTfStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name);
};
