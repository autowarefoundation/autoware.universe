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

#include <tier4_debug_msgs/msg/time_node.hpp>
#include <tier4_debug_msgs/msg/time_tree.hpp>

#include <fmt/format.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::universe_utils
{
/**
 * @brief Class representing a node in the time tracking tree
 */
class TimeNode : public std::enable_shared_from_this<TimeNode>
{
public:
  /**
   * @brief Construct a new Time Node object
   *
   * @param name Name of the node
   */
  explicit TimeNode(const std::string & name);

  /**
   * @brief Add a child node
   *
   * @param name Name of the child node
   * @return std::shared_ptr<TimeNode> Shared pointer to the newly added child node
   */
  std::shared_ptr<TimeNode> add_child(const std::string & name);

  /**
   * @brief Get the result string representing the node and its children in a tree structure
   *
   * @param prefix Prefix for the node (used for indentation)
   * @return std::string Result string representing the node and its children
   */
  std::string get_result_str(const std::string & prefix = "") const;

  /**
   * @brief Construct a TimeTree message from the node and its children
   *
   * @param time_tree_msg Reference to the TimeTree message to be constructed
   * @param parent_id ID of the parent node
   */
  void construct_time_tree_msg(
    tier4_debug_msgs::msg::TimeTree & time_tree_msg, const int parent_id = -1);

  /**
   * @brief Get the parent node
   *
   * @return std::shared_ptr<TimeNode> Shared pointer to the parent node
   */
  std::shared_ptr<TimeNode> get_parent_node() const;

  /**
   * @brief Get the child nodes
   *
   * @return std::vector<std::shared_ptr<TimeNode>> Vector of shared pointers to the child nodes
   */
  std::vector<std::shared_ptr<TimeNode>> get_child_nodes() const;

  /**
   * @brief Set the processing time for the node
   *
   * @param processing_time Processing time to be set
   */
  void set_time(const double processing_time);

  /**
   * @brief Get the name of the node
   *
   * @return std::string Name of the node
   */
  std::string get_name() const;

private:
  const std::string name_;
  double processing_time_{0.0};
  std::shared_ptr<TimeNode> parent_node_{nullptr};
  std::vector<std::shared_ptr<TimeNode>> child_nodes_;
};

/**
 * @brief Class for tracking and reporting the processing time of various functions
 */
class TimeKeeper
{
public:
  /**
   * @brief Construct a new TimeKeeper object
   *
   * @param node Pointer to the ROS2 node
   */
  explicit TimeKeeper(rclcpp::Node * node);

  /**
   * @brief Report the processing times and optionally show them on the terminal
   *
   * @param show_on_terminal Flag indicating whether to show the results on the terminal
   */
  void report(const bool show_on_terminal = false);

  /**
   * @brief Start tracking the processing time of a function
   *
   * @param func_name Name of the function to be tracked
   */
  void start_track(const std::string & func_name);

  /**
   * @brief End tracking the processing time of a function
   *
   * @param func_name Name of the function to end tracking
   */
  void end_track(const std::string & func_name);

private:
  rclcpp::Publisher<tier4_debug_msgs::msg::TimeTree>::SharedPtr
    processing_time_pub_;                        //!< Publisher for the processing time message
  std::shared_ptr<TimeNode> current_time_node_;  //!< Shared pointer to the current time node
  std::shared_ptr<TimeNode> root_node_;          //!< Shared pointer to the root time node
  autoware::universe_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;  //!< StopWatch object for tracking the processing time
};

/**
 * @brief Class for automatically tracking the processing time of a function within a scope
 */
class ScopedStopWatch
{
public:
  /**
   * @brief Construct a new ScopedStopWatch object
   *
   * @param func_name Name of the function to be tracked
   * @param time_keepr Reference to the TimeKeeper object
   */
  ScopedStopWatch(const std::string & func_name, TimeKeeper & time_keepr);

  /**
   * @brief Destroy the ScopedStopWatch object, ending the tracking of the function
   */
  ~ScopedStopWatch();

private:
  const std::string func_name_;
  TimeKeeper & time_keepr_;
};

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__SYSTEM__TIME_KEEPER_HPP_
