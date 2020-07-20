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

#include <autoware_state_monitor/autoware_state_monitor_node.h>

#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <autoware_state_monitor/rosconsole_wrapper.h>

namespace
{
template <class Config>
std::vector<Config> getConfigs(const ros::NodeHandle & nh, const std::string & config_name)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(config_name, xml)) {
    const auto msg = std::string("no parameter found: ") + config_name;
    throw std::runtime_error(msg);
  }

  std::vector<Config> configs;
  configs.reserve(xml.size());

  for (size_t i = 0; i < xml.size(); ++i) {
    auto & value = xml[i];
    configs.emplace_back(value);
  }

  return configs;
}

double calcTopicRate(const std::deque<ros::Time> & topic_received_time_buffer)
{
  assert(topic_received_time_buffer.size() >= 2);

  const auto & buf = topic_received_time_buffer;
  const auto time_diff = buf.back() - buf.front();

  return static_cast<double>(buf.size() - 1) / time_diff.toSec();
}

geometry_msgs::PoseStamped::ConstPtr getCurrentPose(const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0));
  } catch (tf2::TransformException ex) {
    return nullptr;
  }

  geometry_msgs::PoseStamped::Ptr p(new geometry_msgs::PoseStamped());
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return geometry_msgs::PoseStamped::ConstPtr(p);
}

}  // namespace

void AutowareStateMonitorNode::onAutowareEngage(const std_msgs::Bool::ConstPtr & msg)
{
  state_input_.autoware_engage = msg;
}

void AutowareStateMonitorNode::onVehicleControlMode(
  const autoware_vehicle_msgs::ControlMode::ConstPtr & msg)
{
  state_input_.vehicle_control_mode = msg;
}

void AutowareStateMonitorNode::onRoute(const autoware_planning_msgs::Route::ConstPtr & msg)
{
  state_input_.route = msg;

  // Get goal pose
  {
    geometry_msgs::Pose::Ptr p(new geometry_msgs::Pose());
    *p = msg->goal_pose;
    state_input_.goal_pose = geometry_msgs::Pose::ConstPtr(p);
  }

  if (disengage_on_route_) {
    setDisengage();
  }
}

void AutowareStateMonitorNode::onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  state_input_.twist = msg;

  state_input_.twist_buffer.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = msg->header.stamp - state_input_.twist_buffer.front()->header.stamp;

    if (time_diff.toSec() < state_param_.th_stopped_time_sec) {
      break;
    }

    state_input_.twist_buffer.pop_front();
  }
}

void AutowareStateMonitorNode::onTimer(const ros::TimerEvent & event)
{
  // Prepare state input
  state_input_.current_pose = getCurrentPose(tf_buffer_);

  state_input_.topic_stats = getTopicStats();
  state_input_.param_stats = getParamStats();
  state_input_.tf_stats = getTfStats();

  // Update state
  const auto prev_autoware_state = state_machine_->getCurrentState();
  const auto autoware_state = state_machine_->updateState(state_input_);

  if (autoware_state != prev_autoware_state) {
    ROS_INFO(
      "state changed: %s -> %s", toString(prev_autoware_state).c_str(),
      toString(autoware_state).c_str());
  }

  // Disengage on event
  if (disengage_on_complete_ && autoware_state == AutowareState::ArrivedGoal) {
    setDisengage();
  }

  if (disengage_on_error_ && autoware_state == AutowareState::Error) {
    setDisengage();
  }

  // Publish state message
  {
    autoware_system_msgs::AutowareState autoware_state_msg;
    autoware_state_msg.state = toString(autoware_state);

    // Add messages line by line
    std::ostringstream oss;

    for (const auto & msg : state_machine_->getErrorMessages()) {
      oss << msg << std::endl;
    }

    autoware_state_msg.msg = oss.str();

    pub_autoware_state_.publish(autoware_state_msg);
  }

  // Publish diag message
  updater_.force_update();

  // Reset when error
  if (autoware_state == AutowareState::Error) {
    ROS_INFO("reset state machine due to an error");
    state_machine_ = std::make_shared<StateMachine>(state_param_);
  }
}

void AutowareStateMonitorNode::onTopic(
  const topic_tools::ShapeShifter::ConstPtr & msg, const std::string & topic_name)
{
  const auto now = ros::Time::now();

  auto & buf = topic_received_time_buffer_.at(topic_name);
  buf.push_back(now);

  constexpr size_t topic_received_time_buffer_size = 10;
  if (buf.size() > topic_received_time_buffer_size) {
    buf.pop_front();
  }
}

void AutowareStateMonitorNode::registerTopicCallback(const std::string & topic_name)
{
  // Initialize buffer
  topic_received_time_buffer_[topic_name] = {};

  // Register callback
  using Callback = boost::function<void(const topic_tools::ShapeShifter::ConstPtr &)>;
  const auto callback =
    static_cast<Callback>(boost::bind(&AutowareStateMonitorNode::onTopic, this, _1, topic_name));
  sub_topic_map_[topic_name] = nh_.subscribe(topic_name, 10, callback);
}

TopicStats AutowareStateMonitorNode::getTopicStats() const
{
  TopicStats topic_stats;
  topic_stats.checked_time = ros::Time::now();

  for (const auto & topic_config : topic_configs_) {
    // Alias
    const auto & buf = topic_received_time_buffer_.at(topic_config.name);

    // Check at least once received
    if (buf.empty()) {
      topic_stats.non_received_list.push_back(topic_config);
      continue;
    }

    // Check timeout
    const auto last_received_time = buf.back();
    const auto time_diff = (topic_stats.checked_time - last_received_time).toSec();
    const auto is_timeout = (topic_config.timeout != 0) && (time_diff > topic_config.timeout);
    if (is_timeout) {
      topic_stats.timeout_list.emplace_back(topic_config, last_received_time);
    }

    // Check topic rate
    if (!is_timeout && buf.size() >= 2) {
      const auto topic_rate = calcTopicRate(buf);
      if (topic_config.warn_rate != 0 && topic_rate < topic_config.warn_rate) {
        topic_stats.slow_rate_list.emplace_back(topic_config, topic_rate);
      }
    }
  }

  return topic_stats;
}

ParamStats AutowareStateMonitorNode::getParamStats() const
{
  ParamStats param_stats;
  param_stats.checked_time = ros::Time::now();

  for (const auto & param_config : param_configs_) {
    XmlRpc::XmlRpcValue xml;
    const auto result = nh_.getParam(param_config.name, xml);
    if (!result) {
      param_stats.non_set_list.push_back(param_config);
    }
  }

  return param_stats;
}

TfStats AutowareStateMonitorNode::getTfStats() const
{
  TfStats tf_stats;
  tf_stats.checked_time = ros::Time::now();

  for (const auto & tf_config : tf_configs_) {
    try {
      const auto transform =
        tf_buffer_.lookupTransform(tf_config.from, tf_config.to, ros::Time(0), ros::Duration(0));

      const auto last_received_time = transform.header.stamp;
      const auto time_diff = (tf_stats.checked_time - last_received_time).toSec();
      if (time_diff > tf_config.timeout) {
        tf_stats.timeout_list.emplace_back(tf_config, last_received_time);
      }
    } catch (tf2::TransformException ex) {
      tf_stats.non_received_list.push_back(tf_config);
    }
  }

  return tf_stats;
}

void AutowareStateMonitorNode::setDisengage()
{
  std_msgs::Bool msg;
  msg.data = false;
  pub_autoware_engage_.publish(msg);
}

AutowareStateMonitorNode::AutowareStateMonitorNode()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  private_nh_.param("disengage_on_route", disengage_on_route_, true);
  private_nh_.param("disengage_on_complete", disengage_on_complete_, false);
  private_nh_.param("disengage_on_error", disengage_on_error_, false);

  // Parameter for StateMachine
  private_nh_.param("th_arrived_distance_m", state_param_.th_arrived_distance_m, 1.0);
  private_nh_.param("th_stopped_time_sec", state_param_.th_stopped_time_sec, 1.0);
  private_nh_.param("th_stopped_velocity_mps", state_param_.th_stopped_velocity_mps, 0.01);

  // State Machine
  state_machine_ = std::make_shared<StateMachine>(state_param_);

  // Config
  topic_configs_ = getConfigs<TopicConfig>(private_nh_, "topic_configs");
  param_configs_ = getConfigs<ParamConfig>(private_nh_, "param_configs");
  tf_configs_ = getConfigs<TfConfig>(private_nh_, "tf_configs");

  // Topic Callback
  for (const auto & topic_config : topic_configs_) {
    registerTopicCallback(topic_config.name);
  }

  // Subscriber
  sub_autoware_engage_ = private_nh_.subscribe(
    "input/autoware_engage", 1, &AutowareStateMonitorNode::onAutowareEngage, this);
  sub_vehicle_control_mode_ = private_nh_.subscribe(
    "input/vehicle_control_mode", 1, &AutowareStateMonitorNode::onVehicleControlMode, this);
  sub_route_ = private_nh_.subscribe("input/route", 1, &AutowareStateMonitorNode::onRoute, this);
  sub_twist_ = private_nh_.subscribe("input/twist", 100, &AutowareStateMonitorNode::onTwist, this);

  // Publisher
  pub_autoware_state_ =
    private_nh_.advertise<autoware_system_msgs::AutowareState>("output/autoware_state", 1);
  pub_diag_array_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("output/diag_array", 1);
  pub_autoware_engage_ = private_nh_.advertise<std_msgs::Bool>("output/autoware_engage", 1);

  // Diagnostic Updater
  setupDiagnosticUpdater();

  // Wait for first topics
  ros::Duration(1.0).sleep();

  // Timer
  timer_ =
    private_nh_.createTimer(ros::Rate(update_rate_), &AutowareStateMonitorNode::onTimer, this);
}
