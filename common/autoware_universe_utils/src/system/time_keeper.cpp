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

#include "autoware/universe_utils/system/time_keeper.hpp"

#include <iostream>
#include <stdexcept>

namespace autoware::universe_utils
{

TimeNode::TimeNode(const std::string & name) : name_(name)
{
}

std::shared_ptr<TimeNode> TimeNode::add_child(const std::string & name)
{
  auto new_child_node = std::make_shared<TimeNode>(name);
  new_child_node->parent_node_ = shared_from_this();
  child_nodes_.push_back(new_child_node);
  return new_child_node;
}

std::string TimeNode::get_result_str(const std::string & prefix) const
{
  std::ostringstream oss;
  oss << prefix << name_ << " (" << processing_time_ << "ms)\n";
  for (const auto & child : child_nodes_) {
    oss << child->get_result_str(prefix + "  ");
  }
  return oss.str();
}

void TimeNode::construct_time_tree_msg(
  tier4_debug_msgs::msg::TimeTree & time_tree_msg, const int parent_id)
{
  auto time_node_msg = std::make_shared<tier4_debug_msgs::msg::TimeNode>();
  time_node_msg->name = name_;
  time_node_msg->processing_time = processing_time_;
  time_node_msg->id = time_tree_msg.nodes.size();
  time_node_msg->parent_id = parent_id;
  time_tree_msg.nodes.push_back(*time_node_msg);

  for (const auto & child : child_nodes_) {
    child->construct_time_tree_msg(time_tree_msg, time_node_msg->id);
  }
}

std::shared_ptr<TimeNode> TimeNode::get_parent_node() const
{
  return parent_node_;
}
std::vector<std::shared_ptr<TimeNode>> TimeNode::get_child_nodes() const
{
  return child_nodes_;
}
void TimeNode::set_time(const double processing_time)
{
  processing_time_ = processing_time;
}
std::string TimeNode::get_name() const
{
  return name_;
}

TimeKeeper::TimeKeeper(rclcpp::Node * node) : current_time_node_(nullptr)
{
  processing_time_pub_ =
    node->create_publisher<tier4_debug_msgs::msg::TimeTree>("~/debug/processing_time_ms_detail", 1);
}

void TimeKeeper::report(const bool show_on_terminal)
{
  if (current_time_node_ != nullptr) {
    throw std::runtime_error(fmt::format(
      "You must call end_track({}) first, but report() is called", current_time_node_->get_name()));
  }
  if (show_on_terminal) {
    std::cerr << "========================================" << std::endl;
    std::cerr << root_node_->get_result_str() << std::endl;
  }

  tier4_debug_msgs::msg::TimeTree time_tree_msg;
  root_node_->construct_time_tree_msg(time_tree_msg);
  processing_time_pub_->publish(time_tree_msg);

  current_time_node_.reset();
  root_node_.reset();
}

void TimeKeeper::start_track(const std::string & func_name)
{
  if (current_time_node_ == nullptr) {
    current_time_node_ = std::make_shared<TimeNode>(func_name);
    root_node_ = current_time_node_;
  } else {
    current_time_node_ = current_time_node_->add_child(func_name);
  }
  stop_watch_.tic(func_name);
}

void TimeKeeper::end_track(const std::string & func_name)
{
  if (current_time_node_->get_name() != func_name) {
    throw std::runtime_error(fmt::format(
      "You must call end_track({}) first, but end_track({}) is called",
      current_time_node_->get_name(), func_name));
  }
  const double processing_time = stop_watch_.toc(func_name);
  current_time_node_->set_time(processing_time);
  current_time_node_ = current_time_node_->get_parent_node();
}

ScopedStopWatch::ScopedStopWatch(const std::string & func_name, TimeKeeper & time_keepr)
: func_name_(func_name), time_keepr_(time_keepr)
{
  time_keepr_.start_track(func_name_);
}

ScopedStopWatch::~ScopedStopWatch()
{
  time_keepr_.end_track(func_name_);
}

inline ScopedStopWatch TimeKeeper::track(const std::string & func_name)
{
  return ScopedStopWatch(func_name, *this);
}

}  // namespace autoware::universe_utils
