// Copyright 2024 TIER IV, Inc.
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

#include "processing_time_checker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::processing_time_checker
{

namespace
{
std::string remove_last_name(const std::string & str)
{
  return str.substr(0, str.find_last_of("/"));
}

std::string get_last_name(const std::string & str)
{
  return str.substr(str.find_last_of("/") + 1);
}
}  // namespace

ProcessingTimeChecker::ProcessingTimeChecker(const rclcpp::NodeOptions & node_options)
: Node("processing_time_checker", node_options)
{
  const double update_rate = declare_parameter<double>("update_rate");
  const auto processing_time_topic_name_list =
    declare_parameter<std::vector<std::string>>("processing_time_topic_name_list");

  for (const auto & processing_time_topic_name : processing_time_topic_name_list) {
    std::optional<std::string> module_name{std::nullopt};

    // extract module name from topic name
    auto tmp_topic_name = processing_time_topic_name;
    for (size_t i = 0; i < 4; ++i) {  // 4 is enouh for the search depth
      tmp_topic_name = remove_last_name(tmp_topic_name);
      const auto module_name_candidate = get_last_name(tmp_topic_name);
      // clang-format off
      if (
        module_name_candidate != "processing_time_ms" && module_name_candidate != "debug" &&
        module_name_candidate != "total_time")
      {
        module_name = module_name_candidate;
        break;
      }
      // clang-format on
    }

    // register module name
    if (module_name) {
      module_name_map_.insert_or_assign(processing_time_topic_name, *module_name);
    } else {
      throw std::invalid_argument("The format of the processing time topic name is not correct.");
    }
  }

  // create subscribers
  for (const auto & processing_time_topic_name : processing_time_topic_name_list) {
    const auto & module_name = module_name_map_.at(processing_time_topic_name);

    // clang-format off
    processing_time_subscribers_.push_back(
      create_subscription<Float64Stamped>(
        processing_time_topic_name, 1,
        [this, &module_name]([[maybe_unused]] const Float64Stamped & msg) {
          processing_time_map_.insert_or_assign(module_name, msg.data);
        }));
    // clang-format on
  }

  metrics_pub_ = create_publisher<MetricArrayMsg>("~/metrics", 1);

  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ProcessingTimeChecker::on_timer, this));
}

void ProcessingTimeChecker::on_timer()
{
  // create MetricArrayMsg
  MetricArrayMsg metrics_msg;
  for (const auto & processing_time_iterator : processing_time_map_) {
    const auto processing_time_topic_name = processing_time_iterator.first;
    const double processing_time = processing_time_iterator.second;

    // generate MetricMsg
    MetricMsg metric;
    metric.name = "processing_time/" + processing_time_topic_name;
    metric.value = std::to_string(processing_time);
    metric.unit = "millisecond";
    metrics_msg.metric_array.push_back(metric);
  }

  // publish
  metrics_msg.stamp = now();
  metrics_pub_->publish(metrics_msg);
}
}  // namespace autoware::processing_time_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::processing_time_checker::ProcessingTimeChecker)
