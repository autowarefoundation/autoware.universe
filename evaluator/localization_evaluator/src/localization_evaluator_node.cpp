// Copyright 2021 Tier IV, Inc.
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

#include "localization_evaluator/localization_evaluator_node.hpp"

#include "boost/lexical_cast.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace localization_diagnostics
{
LocalizationEvaluatorNode::LocalizationEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("localization_evaluator", node_options),
  odom_sub_(this, "~/input/localization", rclcpp::QoS{1}.get_rmw_qos_profile()),
  odom_gt_sub_(this, "~/input/localization/gt", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), odom_sub_, odom_gt_sub_)
{
  tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  sync_.registerCallback(std::bind(
    &LocalizationEvaluatorNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  output_file_str_ = declare_parameter<std::string>("output_file");
  if (output_file_str_.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "Output file not specified, the results will NOT be saved!"
      "Provide output_file parameter to store the results.");
  }
  // List of metrics to calculate
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);
  for (const std::string & selected_metric :
       declare_parameter<std::vector<std::string>>("selected_metrics")) {
    Metric metric = str_to_metric.at(selected_metric);
    metrics_dict_[metric] = Stat<double>();
    metrics_.push_back(metric);
  }
}

LocalizationEvaluatorNode::~LocalizationEvaluatorNode()
{
  if (!output_file_str_.empty()) {
    // column width is the maximum size we might print + 1 for the space between columns
    // const auto column_width = std::to_string(std::numeric_limits<double>::max()).size() + 1;
    // Write data using format
    std::ofstream f(output_file_str_);
    f << std::fixed << std::left;
    // header
    f << "#Stamp(ns)";
    for (Metric metric : metrics_) {
      f << " " << metric_descriptions.at(metric);
      f << " . .";  // extra "columns" to align columns headers
    }
    f << std::endl;
    f << "#.";
    for (Metric metric : metrics_) {
      (void)metric;
      f << " min max mean";
    }
    f << std::endl;
    // data
    for (size_t i = 0; i < stamps_.size(); ++i) {
      f << stamps_[i].nanoseconds();
      for (Metric metric : metrics_) {
        const auto & stat = metric_stats_[static_cast<size_t>(metric)][i];
        f << " " << stat;
      }
      f << std::endl;
    }
    f.close();
  }
}

DiagnosticStatus LocalizationEvaluatorNode::generateDiagnosticStatus(
  const Metric & metric, const Stat<double> & metric_stat) const
{
  DiagnosticStatus status;
  status.level = status.OK;
  status.name = metric_to_str.at(metric);
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "min";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.min());
  status.values.push_back(key_value);
  key_value.key = "max";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.max());
  status.values.push_back(key_value);
  key_value.key = "mean";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.mean());
  status.values.push_back(key_value);
  return status;
}

void LocalizationEvaluatorNode::syncCallback(
  const Odometry::ConstSharedPtr & msg, const Odometry::ConstSharedPtr & msg_gt)
{
  RCLCPP_INFO(
    get_logger(), "Received two messages at time stamps: %d.%d and %d.%d", msg->header.stamp.sec,
    msg->header.stamp.nanosec, msg_gt->header.stamp.sec, msg->header.stamp.nanosec);

  DiagnosticArray metrics_msg;
  metrics_msg.header.stamp = now();

  geometry_msgs::msg::Point pos(msg->pose.pose.position);
  geometry_msgs::msg::Point pos_gt(msg_gt->pose.pose.position);

  for (Metric metric : metrics_) {
    metrics_dict_[metric] =
      metrics_calculator_.updateStat(metric, pos, pos_gt, metrics_dict_[metric]);
    if (metrics_dict_[metric].count() > 0) {
      metrics_msg.status.push_back(generateDiagnosticStatus(metric, metrics_dict_[metric]));
    }
  }
  if (!metrics_msg.status.empty()) {
    std::cout << "Data: " << metrics_msg.status[0].name << " size: " << metrics_msg.status.size()
              << std::endl;
    metrics_pub_->publish(metrics_msg);
  }
}

void LocalizationEvaluatorNode::onOdom(
  const Odometry::ConstSharedPtr & msg, const Odometry::ConstSharedPtr & msg_gt)
{
  DiagnosticArray metrics_msg;
  metrics_msg.header.stamp = now();

  geometry_msgs::msg::Point pos(msg->pose.pose.position);
  geometry_msgs::msg::Point pos_gt(msg_gt->pose.pose.position);

  for (Metric metric : metrics_) {
    metrics_dict_[metric] =
      metrics_calculator_.updateStat(metric, pos, pos_gt, metrics_dict_[metric]);
    if (metrics_dict_[metric].count() > 0) {
      metrics_msg.status.push_back(generateDiagnosticStatus(metric, metrics_dict_[metric]));
    }
  }
  if (!metrics_msg.status.empty()) {
    std::cout << "Data: " << metrics_msg.status[0].name << " size: " << metrics_msg.status.size()
              << std::endl;
    metrics_pub_->publish(metrics_msg);
  }
}

geometry_msgs::msg::Pose LocalizationEvaluatorNode::getCurrentEgoPose() const
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  geometry_msgs::msg::Pose p;
  try {
    tf_current_pose = tf_buffer_ptr_->lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return p;
  }

  p.orientation = tf_current_pose.transform.rotation;
  p.position.x = tf_current_pose.transform.translation.x;
  p.position.y = tf_current_pose.transform.translation.y;
  p.position.z = tf_current_pose.transform.translation.z;
  return p;
}

}  // namespace localization_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization_diagnostics::LocalizationEvaluatorNode)
