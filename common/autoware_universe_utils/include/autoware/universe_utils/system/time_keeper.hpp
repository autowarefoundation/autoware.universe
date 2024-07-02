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

#ifndef AUTOWARE__UNIVERSE_UTILS__SYSTEM__TIME_KEEPER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__SYSTEM__TIME_KEEPER_HPP_

#include "autoware/universe_utils/system/stop_watch.hpp"

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::universe_utils
{
class TimeNode : public std::enable_shared_from_this<TimeNode>
{
public:
  explicit TimeNode(const std::string & name) : name_(name) {}

  std::shared_ptr<TimeNode> add_child(const std::string & name)
  {
    auto new_child_node = std::make_shared<TimeNode>(name);

    // connect to each other of parent/child
    new_child_node->parent_node_ = shared_from_this();
    child_nodes_.push_back(new_child_node);

    return new_child_node;
  }

  std::string get_result_text() const
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << processing_time_;
    return name_ + ":= " + ss.str() + " [ms]";
  }
  std::shared_ptr<TimeNode> get_parent_node() const { return parent_node_; }
  std::vector<std::shared_ptr<TimeNode>> get_child_nodes() const { return child_nodes_; }

  void set_time(const double processing_time) { processing_time_ = processing_time; }

private:
  const std::string name_{""};
  double processing_time_{0.0};
  std::shared_ptr<TimeNode> parent_node_{nullptr};
  std::vector<std::shared_ptr<TimeNode>> child_nodes_{};
};

// TODO(murooka) has to be refactored
class TimeTree
{
public:
  std::shared_ptr<TimeNode> current_time_node{};
};

class AutoStopWatch
{
public:
  AutoStopWatch(const std::string & func_name, std::shared_ptr<TimeTree> time_tree)
  : func_name_(func_name), time_tree_(time_tree)
  {
    const auto new_time_node = time_tree_->current_time_node->add_child(func_name);
    time_tree_->current_time_node = new_time_node;
    stop_watch_.tic(func_name_);
  }
  ~AutoStopWatch()
  {
    const double processing_time = stop_watch_.toc(func_name_);
    time_tree_->current_time_node->set_time(processing_time);
    time_tree_->current_time_node = time_tree_->current_time_node->get_parent_node();
  }

private:
  const std::string func_name_;
  autoware::universe_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  std::shared_ptr<TimeTree> time_tree_;
};

class TimeKeeper
{
public:
  explicit TimeKeeper(rclcpp::Node * node)
  {
    processing_time_pub_ =
      node->create_publisher<std_msgs::msg::String>("~/debug/processing_time_ms_detail", 1);
  }

  void start(const std::string & func_name)
  {
    // init
    time_tree_ = std::make_shared<TimeTree>();
    time_tree_->current_time_node = std::make_shared<TimeNode>("root");

    // tic
    manual_stop_watch_map_.emplace(
      func_name, std::make_shared<AutoStopWatch>(func_name, time_tree_));
  }
  void end(const std::string & func_name, const bool show_on_terminal = false)
  {
    // toc
    manual_stop_watch_map_.erase(func_name);

    // get result text
    const auto & root_node = time_tree_->current_time_node;
    std::string processing_time_str;
    get_child_info(root_node->get_child_nodes().front(), 0, processing_time_str);
    processing_time_str.pop_back();  // remove last endline.

    // show on the terminal
    if (show_on_terminal) {
      std::cerr << "========================================" << std::endl;
      std::cerr << processing_time_str << std::endl;
    }

    // publish
    std_msgs::msg::String processing_time_msg;
    processing_time_msg.data = processing_time_str;
    processing_time_pub_->publish(processing_time_msg);
  }

  void start_track(const std::string & func_name)
  {
    manual_stop_watch_map_.emplace(
      func_name, std::make_shared<AutoStopWatch>(func_name, time_tree_));
  }
  void end_track(const std::string & func_name) { manual_stop_watch_map_.erase(func_name); }
  AutoStopWatch track(const std::string & func_name) const
  {
    return AutoStopWatch{func_name, time_tree_};
  }

private:
  void get_child_info(
    const std::shared_ptr<TimeNode> time_node, const size_t indent_length,
    std::string & processing_time_str) const
  {
    processing_time_str += std::string(indent_length, ' ') + time_node->get_result_text() + '\n';
    for (const auto & child_time_node : time_node->get_child_nodes()) {
      get_child_info(child_time_node, indent_length + 1, processing_time_str);
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr processing_time_pub_;
  std::shared_ptr<TimeTree> time_tree_;
  std::unordered_map<std::string, std::shared_ptr<AutoStopWatch>> manual_stop_watch_map_;
};
}  // namespace autoware::universe_utils
#endif  // AUTOWARE__UNIVERSE_UTILS__SYSTEM__TIME_KEEPER_HPP_
